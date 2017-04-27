#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import imutils
import cv2
import math

do_visualization_img_proc = True
do_visualization_final_result = True

# detection parameters
greenLower = (19, 0, 0)
greenUpper = (55, 255, 255)
min_area = 0 # pixels
max_contours = 10

dialation_radius_correction = 0.5
min_contour_area_percentage = 0.05

min_radius_diff = 0.1 # percent in terms of ball diameter
min_dist_diff = 0.1 # percent

min_radius = 6 # pixels
min_fill_score = 0.05 # percent

canny_score_mean = 0.7
canny_score_sigma = 0.15
fill_score_mean = 0.8
fill_score_sigma = 0.15

pos_diff_score_mean = 0
pos_diff_score_sigma = 4
dia_diff_score_mean = 0
dia_diff_score_sigma = 2

KF_Q_matrix = np.mat([[8, 0, 0, 0, 0, 0],
                      [0, 4, 0, 0, 0, 0],
                      [0, 0, 8, 0, 0, 0],
                      [0, 0, 0, 4, 0, 0],
                      [0, 0, 0, 0, 2, 0],
                      [0, 0, 0, 0, 0, 1]])

KF_R_matrix = np.mat([[3, 0, 0],
                      [0, 3, 0],
                      [0, 0, 10]])


pixel_size = 7.26 * 10**(-6) # in m
focal_length = 8 * 10**(-3) # in m
tennis_ball_size = 6.54 * 10**(-2) # in m
image_width = 664
image_height = 524


def normal_pdf(x, u, s):
    ret = (1 / math.sqrt(2 * s**2 * math.pi)) * math.exp(-(x - u)**2 / (2 * s**2))
    if ret < 1e-15:
        ret = 1e-15
    return ret

class Hypothesis:
    def __init__(self, contours):
        self.contours = []
        self.contours_cat = None
        self.x = 0
        self.y = 0
        self.r = 0
        self.canny_score = 0 # canny coverage along the diameter of the circle
        self.fill_score = 0 # percentage of the circle that is occupied
        self.color_score = 0

        self.is_garbage = False

        self.log_likelihood = 0 # current frame probability
        self.log_prior = 0
        self.log_probability = 0
        self.probability = 0

        # generate circle
        self.contours = contours
        self.contours_cat = np.concatenate(self.contours)
        ((self.x, self.y), self.r) = cv2.minEnclosingCircle(self.contours_cat)
        self.contour_area = 0

        for contour in contours:
            self.contour_area += cv2.contourArea(contour)

        if (self.r < min_radius):
            self.is_garbage = True

        self.r -= dialation_radius_correction


    def is_duplicate(self, hypotheses):
        for h2 in hypotheses:
            radius_diff = abs(self.r - h2.r) / h2.r
            dist_diff = math.sqrt((self.x - h2.x)**2 + (self.y - h2.y)**2) / (h2.r * 2)
            if dist_diff < min_dist_diff and radius_diff < min_radius_diff:
                return True
        return False

    def __str__(self):
        return "((%d,%d),%d,%f,%f|%f,%f,%f)" % (self.x, self.y, self.r, self.canny_score, self.fill_score,self.log_likelihood,self.log_prior,self.log_probability)

    def __repr__(self):
        return str(self)


class Ball_tracking:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_color", Image, self.callback, queue_size=1)
        self.initialized = False
        self.image = None
        self.curr_hypotheses = []
        self.past_hypotheses = []
        self.best_hypothesis = None
        self.num_iterations = 0

        self.ts_curr = 0
        self.ts_prev = 0
        self.dt = 0

        # for the Kalman Filter stuff
        self.x_k = np.mat("0; 0; 0; 0; 0; 0")
        self.x_k_prev = self.x_k;
        self.A = np.mat([[1, 0, 0, 0, 0, 0], 
                         [0, 1, 0, 0, 0, 0], 
                         [0, 0, 1, 0, 0, 0], 
                         [0, 0, 0, 1, 0, 0], 
                         [0, 0, 0, 0, 1, 0], 
                         [0, 0, 0, 0, 0, 1]])
        self.C = np.mat([[1, 0, 0, 0, 0, 0], 
                         [0, 0, 1, 0, 0, 0], 
                         [0, 0, 0, 0, 1, 0]])
        self.Q = KF_Q_matrix
        self.R = KF_R_matrix
        self.P_k = np.mat([[1, 0, 0, 0, 0, 0], 
                           [0, 1, 0, 0, 0, 0], 
                           [0, 0, 1, 0, 0, 0], 
                           [0, 0, 0, 1, 0, 0], 
                           [0, 0, 0, 0, 1, 0], 
                           [0, 0, 0, 0, 0, 1]])
        self.P_k_prev = self.P_k
        self.I_6x6 = np.mat([[1, 0, 0, 0, 0, 0], 
                             [0, 1, 0, 0, 0, 0], 
                             [0, 0, 1, 0, 0, 0], 
                             [0, 0, 0, 1, 0, 0], 
                             [0, 0, 0, 0, 1, 0], 
                             [0, 0, 0, 0, 0, 1]])

    def callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.initialized = True
        except CvBridgeError as e:
            print(e)

    def process(self):

        if not self.initialized:
            return

        self.ts_prev = self.ts_curr
        self.ts_curr = rospy.get_time()
        self.dt = self.ts_curr - self.ts_prev;
        image = self.image.copy()

        if not self.compute_image(image):
            return None

        self.kalman_filter()

        x = self.x_k[0, 0] # image frame x, y
        y = self.x_k[2, 0]
        r = self.x_k[4, 0]
        X, Y, Z, bearing = self.locate_3d(x, y, r) # get 3D coordinate in camera frame

        rospy.loginfo("Tracking: X=%.2f Y=%.2f Z=%.2f bearing=%.2f" % (X, Y, Z, bearing * 180 / math.pi))

        self.num_iterations += 1

        if do_visualization_final_result:
            cv2.circle(image, (int(x), int(y)), int(r), (0, 0, 255), 2) # visualization
            text = "Position=(%.3f,%.3f,%.3f) Bearing=%.3f, Confidence:%.10f%%" % (X, Y, Z, bearing * 180 / math.pi, self.best_hypothesis.probability * 100)
            cv2.putText(image, text, (10, 500), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
            cv2.imshow("Final Results", image)
            cv2.waitKey(3)

        cv2.imwrite("/home/lichunshang/Pictures/tennis_ball frames/" + str(self.num_iterations) + ".png", image)

        return X, Y, Z, bearing

    def locate_3d(self, x, y, r):
        z_ball = (tennis_ball_size * focal_length) / (r * 2 * pixel_size)
        x_ball = z_ball * (x - image_width / 2.0) * pixel_size / focal_length
        y_ball = z_ball * (y - image_height / 2.0) * pixel_size / focal_length
        bearing = math.atan(x_ball / z_ball)

        return x_ball, y_ball, z_ball, bearing

    def kalman_filter(self):

        # initialization
        if self.num_iterations <= 0:
            #initialize the first estimate to the measurement
            self.x_k = np.mat("0; 0; 0; 0; 0; 0")
            self.x_k[0] = self.best_hypothesis.x
            self.x_k[2] = self.best_hypothesis.y
            self.x_k[4] = self.best_hypothesis.r
        else:
            # prediction
            self.A[0, 1] = self.dt
            self.A[2, 3] = self.dt
            self.A[4, 5] = self.dt
            x_k_pred = self.A * self.x_k_prev
            P_k_pred = self.A * self.P_k_prev * self.A.T + self.Q

            #update
            K = P_k_pred * self.C.T * np.linalg.inv(self.C * P_k_pred * self.C.T + self.R)
            y_k = np.mat([[self.best_hypothesis.x], [self.best_hypothesis.y], [self.best_hypothesis.r]])
            self.x_k = x_k_pred + K * (y_k - self.C * x_k_pred)
            self.P_k = (self.I_6x6 - K * self.C) * P_k_pred

        self.x_k_prev = self.x_k
        self.P_k_prev = self.P_k

    def compute_image(self, image):
        # image = self.image.copy()
        canny_edges = cv2.Canny(image, 100, 150);
        # kernel = np.ones((1,1),np.uint8)
        # canny_edges = cv2.dilate(canny_edges, kernel, iterations=1)
        canny_edges = cv2.GaussianBlur(canny_edges, (9,9), 0)
        canny_edges = canny_edges.clip(0, 255 // 3) * 3

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        kernel = np.ones((2,2),np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=3)

        #use colored image for visualization
        if do_visualization_img_proc:
            mask_clr = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
            canny_clr = cv2.cvtColor(canny_edges, cv2.COLOR_GRAY2RGB)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

        # filter the contours by filling in the undesirable ones
        contours = sorted(contours, key=lambda c: cv2.contourArea(c), reverse=True)
        if len(contours) > max_contours:
            contours = contours[0:max_contours]

        filtered_contours = []
        for cnt in contours:        
            area = cv2.contourArea(cnt)      
            if area > min_area:
                filtered_contours.append(cnt)

        #generate hypotheses from the current contours
        initial_hypotheses = []
        for i in range(0, len(filtered_contours)):
            #add two contour hypotheses
            for j in range(i + 1, len(filtered_contours)):
                hypothesis = Hypothesis([filtered_contours[i], filtered_contours[j]])
                if not hypothesis.is_duplicate(initial_hypotheses):
                    initial_hypotheses.append(hypothesis)

            # add single contour hypotheses
            hypothesis = Hypothesis([filtered_contours[i]])
            if not hypothesis.is_duplicate(initial_hypotheses):
                initial_hypotheses.append(hypothesis)

        if len(initial_hypotheses) == 0:
            rospy.logwarn("No hypotheses generated")
            return False

        #compute the current frame metrics
        for h in initial_hypotheses:

            if h.contour_area / (math.pi * h.r**2) < min_contour_area_percentage:
                h.is_garbage = True
                continue

            # calculate canny, percent spacing and colour
            # canny
            circle_mask = np.zeros_like(mask)
            cv2.circle(circle_mask, (int(h.x), int(h.y)), int(h.r), 255, 1) # select the diameter of the circle
            canny_sum = cv2.sumElems(cv2.bitwise_and(circle_mask, canny_edges))[0] # select the first channel
            h.canny_score = canny_sum / (2 * math.pi * h.r) / 255.0 # divide by diameter to normalize for circle size

            # percent empty
            cv2.circle(circle_mask, (int(h.x), int(h.y)), int(h.r), 255, -1) #fill the circle
            # kernel = np.ones((5,5),np.uint8)
            # circle_mask = cv2.dilate(circle_mask, kernel, iterations=1)
            fill_sum = max(cv2.sumElems(cv2.bitwise_and(circle_mask, mask))[0], h.contour_area) / 255.0
            h.fill_score = fill_sum / (math.pi * h.r**2) # divide by area to normalize for circle size

            if (h.fill_score < min_fill_score):
                h.is_garbage = True
                continue

            # TODO color score

        # get rid of garbage hypothesis
        self.curr_hypotheses = []
        for h in initial_hypotheses:
            if not h.is_garbage:
                self.curr_hypotheses.append(h)

                #calculate current probabilities
                canny_score_prob = normal_pdf(h.canny_score, canny_score_mean, canny_score_sigma)
                fill_score_prob = normal_pdf(h.fill_score, fill_score_mean, fill_score_sigma)
                h.log_likelihood = math.log(canny_score_prob) + math.log(fill_score_prob)

                if do_visualization_img_proc:
                    # cv2.circle(image, (int(h.x), int(h.y)), int(h.r), (0, 0, 255), 1) # visualization
                    cv2.circle(mask_clr, (int(h.x), int(h.y)), int(h.r), (0, 0, 255), 1) # visualization
                    cv2.circle(canny_clr, (int(h.x), int(h.y)), int(h.r), (0, 0, 255), 1) # visualization

        # calculate transition probability and prior
        for hc in self.curr_hypotheses:

            if len(self.past_hypotheses) == 0:
                hc.log_prior = math.log(1)
            else:
                sum_transition = 0
                for hp in self.past_hypotheses:
                    pos_diff = math.sqrt((hc.x - hp.x)**2 + (hc.y - hp.y)**2)
                    dia_diff = (hc.r - hp.r) * 2

                    pos_diff_score = pos_diff / (hp.r * 2) # change in dist w.r.t diameter
                    dia_diff_score = dia_diff / (hp.r * 2) # percent change in diameter

                    pos_diff_score_prob = normal_pdf(pos_diff_score, pos_diff_score_mean, pos_diff_score_sigma)
                    dia_diff_score_prob = normal_pdf(dia_diff_score, dia_diff_score_mean, dia_diff_score_sigma)

                    log_transition = math.log(pos_diff_score_prob) +  math.log(dia_diff_score_prob) + hp.log_probability
                    sum_transition += math.exp(log_transition)

                hc.log_prior = math.log(sum_transition)

        # calculate the final probability, and normalization factor
        probability_sum = 0
        for hc in self.curr_hypotheses:
            hc.log_probability = hc.log_prior + hc.log_likelihood
            probability_sum += math.exp(hc.log_probability)

        print(self.curr_hypotheses)

        if (probability_sum < 1e-15):
            rospy.logwarn("Normalization failed probability sum too small")
            return False

        log_probability_sum = math.log(probability_sum)

        for hc in self.curr_hypotheses:
            hc.log_probability -= log_probability_sum
            hc.probability = math.exp(hc.log_probability)

        h_sel = sorted(self.curr_hypotheses, key=lambda h: h.probability, reverse=True)[0]
        rospy.loginfo("Total %d hypotheses, best: (%.2f %.2f, %.2f, %.4f)" % (len(self.curr_hypotheses), h_sel.x, h_sel.y, h_sel.r, h_sel.probability))
        self.best_hypothesis = h_sel

        self.past_hypotheses = self.curr_hypotheses
        self.curr_hypotheses = []

        if do_visualization_img_proc:
            cv2.circle(image, (int(h_sel.x), int(h_sel.y)), int(h_sel.r), (0, 255, 255), 2) # visualization
            cv2.imshow("After filtering", mask_clr)
            cv2.imshow("Input Image", image)
            cv2.imshow("edges", canny_clr)
            cv2.waitKey(3)

        return True

def main():
    node = Ball_tracking()
    rospy.init_node('ball_tracking', anonymous=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.process()
        rate.sleep()
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()