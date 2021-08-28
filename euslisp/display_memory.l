#!/usr/bin/env roseus
(ros::load-ros-manifest "memorization")
(ros::roseus "display_memory")

(require "functions.l")
(defclass display_memory
  :super propertied-object
  :slots (objects-list timer))

(defmethod display_memory
  (:init ()
         (format t "init")
         (ros::subscribe "~slave_rarm_pose" geometry_msgs::PoseStamped #'send self :call-back-r)
         (ros::subscribe "~slave_larm_pose" geometry_msgs::PoseStamped #'send self :call-back-l)
         (ros::subscribe "~recognized_params" memorization::RecognizedParams #'send self :call-back-params))
         ;; (setq timer (instance mtimer :init)))

  (:call-back-params (msg)
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
                                   boxes)))
  ;; (objects objects-list)

  (:add-obj (obj)
            (setq point (make-cube 20 20 20))
            (setq quaternion (float-vector (send (send obj :orientation) :x) (send (send obj :orientation) :y) (send (send obj :orientation) :z) (send (send obj :orientation) :w)))
            (send point :translate (scale 1000 (float-vector (send (send obj :position) :x) (send (send obj :position) :y) (send (send obj :position) :z))))
            (send point :replace-rot (quaternion2matrix (scale (/ 1 (norm quaternion)) quaternion)))
            (setq objects-list (append objects-list point)))

  (:call-back-r (msg)
                (send self :add-obj msg)
                ;; (format t "in rarm-cb~%")
                )

  (:call-back-l (msg)
                (send self :add-obj msg)
                ;; (format t "in larm-cb~%")
                )

  (:display ()
            (objects objects-list)
            (setq objects-list (list))
            )
  )

(setq m (instance display_memory :init))

(do-until-key
 (ros::spin-once)
 (unix:sleep 1)
 )
