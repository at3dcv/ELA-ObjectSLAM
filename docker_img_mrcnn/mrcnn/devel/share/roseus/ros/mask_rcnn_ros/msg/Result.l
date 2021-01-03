;; Auto-generated. Do not edit!


(when (boundp 'mask_rcnn_ros::Result)
  (if (not (find-package "MASK_RCNN_ROS"))
    (make-package "MASK_RCNN_ROS"))
  (shadow 'Result (find-package "MASK_RCNN_ROS")))
(unless (find-package "MASK_RCNN_ROS::RESULT")
  (make-package "MASK_RCNN_ROS::RESULT"))

(in-package "ROS")
;;//! \htmlinclude Result.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mask_rcnn_ros::Result
  :super ros::object
  :slots (_header _boxes _class_ids _class_names _scores _masks ))

(defmethod mask_rcnn_ros::Result
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:boxes __boxes) (let (r) (dotimes (i 0) (push (instance sensor_msgs::RegionOfInterest :init) r)) r))
    ((:class_ids __class_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:class_names __class_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:scores __scores) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:masks __masks) (let (r) (dotimes (i 0) (push (instance sensor_msgs::Image :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _boxes __boxes)
   (setq _class_ids __class_ids)
   (setq _class_names __class_names)
   (setq _scores __scores)
   (setq _masks __masks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:boxes
   (&rest __boxes)
   (if (keywordp (car __boxes))
       (send* _boxes __boxes)
     (progn
       (if __boxes (setq _boxes (car __boxes)))
       _boxes)))
  (:class_ids
   (&optional __class_ids)
   (if __class_ids (setq _class_ids __class_ids)) _class_ids)
  (:class_names
   (&optional __class_names)
   (if __class_names (setq _class_names __class_names)) _class_names)
  (:scores
   (&optional __scores)
   (if __scores (setq _scores __scores)) _scores)
  (:masks
   (&rest __masks)
   (if (keywordp (car __masks))
       (send* _masks __masks)
     (progn
       (if __masks (setq _masks (car __masks)))
       _masks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/RegionOfInterest[] _boxes
    (apply #'+ (send-all _boxes :serialization-length)) 4
    ;; int32[] _class_ids
    (* 4    (length _class_ids)) 4
    ;; string[] _class_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _class_names)) 4
    ;; float32[] _scores
    (* 4    (length _scores)) 4
    ;; sensor_msgs/Image[] _masks
    (apply #'+ (send-all _masks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/RegionOfInterest[] _boxes
     (write-long (length _boxes) s)
     (dolist (elem _boxes)
       (send elem :serialize s)
       )
     ;; int32[] _class_ids
     (write-long (length _class_ids) s)
     (dotimes (i (length _class_ids))
       (write-long (elt _class_ids i) s)
       )
     ;; string[] _class_names
     (write-long (length _class_names) s)
     (dolist (elem _class_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float32[] _scores
     (write-long (length _scores) s)
     (dotimes (i (length _scores))
       (sys::poke (elt _scores i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; sensor_msgs/Image[] _masks
     (write-long (length _masks) s)
     (dolist (elem _masks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/RegionOfInterest[] _boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _boxes (let (r) (dotimes (i n) (push (instance sensor_msgs::RegionOfInterest :init) r)) r))
     (dolist (elem- _boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32[] _class_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _class_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _class_ids i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; string[] _class_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _class_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _class_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float32[] _scores
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _scores (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _scores i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; sensor_msgs/Image[] _masks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _masks (let (r) (dotimes (i n) (push (instance sensor_msgs::Image :init) r)) r))
     (dolist (elem- _masks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get mask_rcnn_ros::Result :md5sum-) "5a75b825f87d437e076ac357fe9ef89c")
(setf (get mask_rcnn_ros::Result :datatype-) "mask_rcnn_ros/Result")
(setf (get mask_rcnn_ros::Result :definition-)
      "std_msgs/Header header

# Bounding boxes in pixels
sensor_msgs/RegionOfInterest[] boxes

# Integer class IDs for each bounding box
int32[] class_ids

# String class IDs for each bouding box
string[] class_names

# Float probability scores of the class_id
float32[] scores

# Instance masks as Image
sensor_msgs/Image[] masks


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the \"raw\"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

")



(provide :mask_rcnn_ros/Result "5a75b825f87d437e076ac357fe9ef89c")


