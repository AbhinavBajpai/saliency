; Auto-generated. Do not edit!


(cl:in-package dynamic-msg)


;//! \htmlinclude Cap.msg.html

(cl:defclass <Cap> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Cap (<Cap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic-msg:<Cap> is deprecated: use dynamic-msg:Cap instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Cap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-msg:name-val is deprecated.  Use dynamic-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <Cap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-msg:msg-val is deprecated.  Use dynamic-msg:msg instead.")
  (msg m))

(cl:ensure-generic-function 'pub-val :lambda-list '(m))
(cl:defmethod pub-val ((m <Cap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-msg:pub-val is deprecated.  Use dynamic-msg:pub instead.")
  (pub m))

(cl:ensure-generic-function 'sub-val :lambda-list '(m))
(cl:defmethod sub-val ((m <Cap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic-msg:sub-val is deprecated.  Use dynamic-msg:sub instead.")
  (sub m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cap>) ostream)
  "Serializes a message object of type '<Cap>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cap>) istream)
  "Deserializes a message object of type '<Cap>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cap>)))
  "Returns string type for a message object of type '<Cap>"
  "dynamic/Cap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cap)))
  "Returns string type for a message object of type 'Cap"
  "dynamic/Cap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cap>)))
  "Returns md5sum for a message object of type '<Cap>"
  "788b9796b5ad2bac96e86245358b39ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cap)))
  "Returns md5sum for a message object of type 'Cap"
  "788b9796b5ad2bac96e86245358b39ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cap>)))
  "Returns full string definition for message of type '<Cap>"
  (cl:format cl:nil "string name~%string msg~%int8 pub~%int8 sub~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cap)))
  "Returns full string definition for message of type 'Cap"
  (cl:format cl:nil "string name~%string msg~%int8 pub~%int8 sub~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cap>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'msg))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cap>))
  "Converts a ROS message object to a list"
  (cl:list 'Cap
    (cl:cons ':name (name msg))
    (cl:cons ':msg (msg msg))
    (cl:cons ':pub (pub msg))
    (cl:cons ':sub (sub msg))
))
