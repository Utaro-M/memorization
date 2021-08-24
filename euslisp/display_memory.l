#!/usr/bin/env roseus
(ros::load-ros-manifest "memorization")
(ros::roseus "display_memory")

(require "functions.l")

(defun call-back-params (msg)
  (format t "in cb-params~%")
  (setq boxes (send msg :box))
  (setq radious (send msg :radious))
  (setq objects-list
        (mapcar #'(lambda (box) (progn
                                  (setq sorted_list (quick-sort (list (send (send box :dimensions) :x) (send (send box :dimensions) :y) (send (send box :dimensions) :z))))
                                  (format t "sorted_list = ~A~%" sorted_list)
                                  (setq cylinder (make-cylinder (* 1000 (/ (+ (elt sorted_list 1) (elt sorted_list 2)) 4)) (* 1000 (elt sorted_list 0))))
                                  (setq quaternion (float-vector (send (send (send box :pose) :orientation) :x) (send (send (send box :pose) :orientation) :y) (send (send (send box :pose) :orientation) :z) (send (send (send box :pose) :orientation) :w)))
                                  (send cylinder :translate (scale 1000 (float-vector (send (send (send box :pose) :position) :x) (send (send (send box :pose) :position) :y) (send (send (send box :pose) :position) :z))))
                                  (send cylinder :replace-rot (quaternion2matrix (scale (/ 1 (norm quaternion)) quaternion)))
                                  cylinder
                                  ))
                    boxes))
  (objects objects-list)
  )

(defun call-back-r (msg)
  ;; (format t "in rarm-cb~%")
  )

(defun call-back-l (msg)
  ;; (format t "in larm-cb~%")
  )

(ros::subscribe "~slave_rarm_pose" geometry_msgs::PoseStamped #'call-back-r)
(ros::subscribe "~slave_larm_pose" geometry_msgs::PoseStamped #'call-back-l)
(ros::subscribe "~recognized_params" memorization::RecognizedParams #'call-back-params)

(do-until-key
 (ros::spin-once)
 )
