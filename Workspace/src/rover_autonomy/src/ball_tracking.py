#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import imutils
import cv2
import math

do_visualization = True

# detection parameters
greenLower = (15, 0, 0)
greenUpper = (75, 255, 255)
min_area = 10 # pixels

min_radius_diff = 0.05 # percent in terms of ball diameter
min_dist_diff = 0.05 # percent

min_radius = 3 # pixels
min_fill_score = 0.05 # percent

canny_score_mean = 1
canny_score_sigma = 0.3
fill_score_mean = 1
fill_score_sigma = 0.3

pos_diff_score_mean = 0
pos_diff_score_sigma = 0.5
dia_diff_score_mean = 0
dia_diff_score_sigma = 0.25


def normal_pdf(x, u, s):
    ret = (1 / math.sqrt(2 * s**2 * math.pi)) * math.exp(-(x - u)**2 / (2 * s**2))
    if ret < 1e-7:
        ret = 1e-7
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

        if (self.r < min_radius):
            self.is_garbage = True

    def is_duplicate(self, hypotheses):
        for h2 in hypotheses:
            radius_diff = abs(self.r - h2.r) / h2.r
            dist_diff = math.sqrt((self.x - h2.x)**2 + (self.y - h2.y)**2) / (h2.r * 2)
            if dist_diff < min_dist_diff and radius_diff < min_radius_diff:
                return True
        return False

    def __str__(self):
        return "((%d,%d),%d,%f,%f)" % (self.x, self.y, self.r, self.canny_score, self.fill_score)

    def __repr__(self):
        return str(self)


class Ball_tracking:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_color", Image, self.callback, queue_size=1)
        self.image = None
        self.curr_hypotheses = []
        self.past_hypotheses = []
        self.num_iterations = 0

    def callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def process(self):
        self.compute_image()

    def locate_3d(self):
        pass

    def kalman_filter(self):
        pass

    def compute_image(self):
        image = self.image.copy()
        canny_edges = cv2.Canny(image, 100, 150);
        kernel = np.ones((3,3),np.uint8)
        canny_edges = cv2.dilate(canny_edges, kernel, iterations=1)
        canny_edges = cv2.GaussianBlur(canny_edges, (7,7), 0)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        kernel = np.ones((2,2),np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        #use colored image for visualization
        if do_visualization:
            mask_clr = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
            canny_clr = cv2.cvtColor(canny_edges, cv2.COLOR_GRAY2RGB)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

        # filter the contours by filling in the undesirable ones
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


        #compute the current frame metrics
        for h in initial_hypotheses:
            # calculate canny, percent spacing and colour
            # canny
            circle_mask = np.zeros_like(mask)
            cv2.circle(circle_mask, (int(h.x), int(h.y)), int(h.r), 255, 1) # select the diameter of the circle
            canny_sum = cv2.sumElems(cv2.bitwise_and(circle_mask, canny_edges))[0] # select the first channel
            h.canny_score = canny_sum / (2 * math.pi * h.r) / 255.0 # divide by diameter to normalize for circle size

            # percent empty
            cv2.circle(circle_mask, (int(h.x), int(h.y)), int(h.r), 255, -1) #fill the circle
            fill_sum = cv2.sumElems(cv2.bitwise_and(circle_mask, mask))[0] / 255.0
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

                if do_visualization:
                    cv2.circle(image, (int(h.x), int(h.y)), int(h.r), (0, 0, 255), 1) # visualization
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

        log_probability_sum = math.log(probability_sum)

        for hc in self.curr_hypotheses:
            hc.log_probability -= log_probability_sum
            hc.probability = math.exp(hc.log_probability)

        h_sel = sorted(self.curr_hypotheses, key=lambda h: h.probability, reverse=True)[0]
        rospy.loginfo("Total %d hypotheses, best: (%d, %d, %d, %.4f)" % (len(self.curr_hypotheses), h_sel.x, h_sel.y, h_sel.r, h_sel.probability))


        self.past_hypotheses = self.curr_hypotheses
        self.curr_hypotheses = []
        self.num_iterations += 1

        if do_visualization:
            cv2.circle(image, (int(h_sel.x), int(h_sel.y)), int(h_sel.r), (0, 255, 255), 3) # visualization
            cv2.imshow("After filtering", mask_clr)
            cv2.imshow("Input Image", image)
            cv2.imshow("edges", canny_clr)
            cv2.waitKey(3)

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