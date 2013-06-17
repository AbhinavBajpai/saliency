; Auto-generated. Do not edit!


(cl:in-package dynamic-srv)


;//! \htmlinclude Cap-request.msg.html

(cl:defclass <Cap-request> (roslisp-msg-protocol:ros-message)
  ((poke
    :reader poke
    :initarg :poke
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Cap-request (<Cap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic-srv:<Cap-request> is deprecated: use dynamic-srv:Cap-request instead.")))

(cl:ensure-generic-function 'poke-val :lambda-list '(m))
(cl:defmethod poke-val ((m <Cap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-srv:poke-val is deprecated.  Use dynamic-srv:poke instead.")
  (poke m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cap-request>) ostream)
  "Serializes a message object of type '<Cap-request>"
  (cl:let* ((signed (cl:slot-value msg 'poke)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cap-request>) istream)
  "Deserializes a message object of type '<Cap-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'poke) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cap-request>)))
  "Returns string type for a service object of type '<Cap-request>"
  "dynamic/CapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cap-request)))
  "Returns string type for a service object of type 'Cap-request"
  "dynamic/CapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cap-request>)))
  "Returns md5sum for a message object of type '<Cap-request>"
  "e0c84457e4f5a08d43be0de82553d460")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cap-request)))
  "Returns md5sum for a message object of type 'Cap-request"
  "e0c84457e4f5a08d43be0de82553d460")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cap-request>)))
  "Returns full string definition for message of type '<Cap-request>"
  (cl:format cl:nil "int8 poke~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cap-request)))
  "Returns full string definition for message of type 'Cap-request"
  (cl:format cl:nil "int8 poke~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cap-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Cap-request
    (cl:cons ':poke (poke msg))
))
;//! \htmlinclude Cap-response.msg.html

(cl:defclass <Cap-response> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform "")
   (pub
    :reader pub
    :initarg :pub
    :type cl:fixnum
    :initform 0)
   (sub
    :reader sub
    :initarg :sub
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Cap-response (<Cap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic-srv:<Cap-response> is deprecated: use dynamic-srv:Cap-response instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Cap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-srv:name-val is deprecated.  Use dynamic-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <Cap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-srv:msg-val is deprecated.  Use dynamic-srv:msg instead.")
  (msg m))

(cl:ensure-generic-function 'pub-val :lambda-list '(m))
(cl:defmethod pub-val ((m <Cap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-srv:pub-val is deprecated.  Use dynamic-srv:pub instead.")
  (pub m))

(cl:ensure-generic-function 'sub-val :lambda-list '(m))
(cl:defmethod sub-val ((m <Cap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-srv:sub-val is deprecated.  Use dynamic-srv:sub instead.")
  (sub m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cap-response>) ostream)
  "Serializes a message object of type '<Cap-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
  (cl:let* ((signed (cl:slot-value msg 'pub)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sub)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cap-response>) istream)
  "Deserializes a message object of type '<Cap-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pub) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sub) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cap-response>)))
  "Returns string type for a service object of type '<Cap-response>"
  "dynamic/CapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cap-response)))
  "Returns string type for a service object of type 'Cap-response"
  "dynamic/CapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cap-response>)))
  "Returns md5sum for a message object of type '<Cap-response>"
  "e0c84457e4f5a08d43be0de82553d460")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cap-response)))
  "Returns md5sum for a message object of type 'Cap-response"
  "e0c84457e4f5a08d43be0de82553d460")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cap-response>)))
  "Returns full string definition for message of type '<Cap-response>"
  (cl:format cl:nil "string name~%string msg~%int8 pub~%int8 sub~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cap-response)))
  "Returns full string definition for message of type 'Cap-response"
  (cl:format cl:nil "string name~%string msg~%int8 pub~%int8 sub~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cap-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'msg))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Cap-response
    (cl:cons ':name (name msg))
    (cl:cons ':msg (msg msg))
    (cl:cons ':pub (pub msg))
    (cl:cons ':sub (sub msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Cap)))
  'Cap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Cap)))
  'Cap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cap)))
  "Returns string type for a service object of type '<Cap>"
  "dynamic/Cap")