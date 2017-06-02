; Auto-generated. Do not edit!


(cl:in-package python_service_test-srv)


;//! \htmlinclude PanoService-request.msg.html

(cl:defclass <PanoService-request> (roslisp-msg-protocol:ros-message)
  ((go
    :reader go
    :initarg :go
    :type cl:integer
    :initform 0))
)

(cl:defclass PanoService-request (<PanoService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PanoService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PanoService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name python_service_test-srv:<PanoService-request> is deprecated: use python_service_test-srv:PanoService-request instead.")))

(cl:ensure-generic-function 'go-val :lambda-list '(m))
(cl:defmethod go-val ((m <PanoService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader python_service_test-srv:go-val is deprecated.  Use python_service_test-srv:go instead.")
  (go m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PanoService-request>) ostream)
  "Serializes a message object of type '<PanoService-request>"
  (cl:let* ((signed (cl:slot-value msg 'go)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PanoService-request>) istream)
  "Deserializes a message object of type '<PanoService-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'go) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PanoService-request>)))
  "Returns string type for a service object of type '<PanoService-request>"
  "python_service_test/PanoServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PanoService-request)))
  "Returns string type for a service object of type 'PanoService-request"
  "python_service_test/PanoServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PanoService-request>)))
  "Returns md5sum for a message object of type '<PanoService-request>"
  "b5eb47b68f24cd3aaa45ca1005960292")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PanoService-request)))
  "Returns md5sum for a message object of type 'PanoService-request"
  "b5eb47b68f24cd3aaa45ca1005960292")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PanoService-request>)))
  "Returns full string definition for message of type '<PanoService-request>"
  (cl:format cl:nil "int32 go~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PanoService-request)))
  "Returns full string definition for message of type 'PanoService-request"
  (cl:format cl:nil "int32 go~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PanoService-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PanoService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PanoService-request
    (cl:cons ':go (go msg))
))
;//! \htmlinclude PanoService-response.msg.html

(cl:defclass <PanoService-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass PanoService-response (<PanoService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PanoService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PanoService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name python_service_test-srv:<PanoService-response> is deprecated: use python_service_test-srv:PanoService-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <PanoService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader python_service_test-srv:result-val is deprecated.  Use python_service_test-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PanoService-response>) ostream)
  "Serializes a message object of type '<PanoService-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PanoService-response>) istream)
  "Deserializes a message object of type '<PanoService-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PanoService-response>)))
  "Returns string type for a service object of type '<PanoService-response>"
  "python_service_test/PanoServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PanoService-response)))
  "Returns string type for a service object of type 'PanoService-response"
  "python_service_test/PanoServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PanoService-response>)))
  "Returns md5sum for a message object of type '<PanoService-response>"
  "b5eb47b68f24cd3aaa45ca1005960292")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PanoService-response)))
  "Returns md5sum for a message object of type 'PanoService-response"
  "b5eb47b68f24cd3aaa45ca1005960292")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PanoService-response>)))
  "Returns full string definition for message of type '<PanoService-response>"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PanoService-response)))
  "Returns full string definition for message of type 'PanoService-response"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PanoService-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PanoService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PanoService-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PanoService)))
  'PanoService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PanoService)))
  'PanoService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PanoService)))
  "Returns string type for a service object of type '<PanoService>"
  "python_service_test/PanoService")