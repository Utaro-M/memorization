#!/usr/bin/env roseus
(ros::load-ros-manifest "rm_debris_vision")
(ros::roseus "display_memory")

(setq points (instance geometry_msgs::PolygonStamped :init))

(ros::subscribe "~slave_rarm_pose" geometry_msgs::PoseStamped #'call-back-r)
(ros::subscribe "~slave_larm_pose" geometry_msgs::PoseStamped #'call-back-l)
(ros::subscribe "~recognizedparams" memorization_msgs::RecognizedParams #'call-back-params)

(defun call-back-params(msg)
  (setq boxes (send msg box))
  (setq radious (send msg radious))
  (mapcar #'(lambda (box) (prong (setq sorted_list (quick-sort (list (send (send box dimensions) x) (send (send box dimensions) y) (send (send box dimensions) z)))) (make-cylinder (+ (elt sorted_list 1) (elt sorted_list 2)) (elt sorted_list 0)
  (while (boxes)
    (setq (make-cylinder radious (send
(do-until-key
 (setq cnt (+ cnt 1))
 ;;平面がない時はリセット
 (if (= cnt 10000000)
     (progn
        (if (equal tmp-point-list (send points :polygon :points))
            (progn
              (print "reset")
              (setq points (instance geometry_msgs::PolygonStamped :init))
              ))
        (setq tmp-point-list (send points :polygon :points))
        (setq cnt 0)))
  (ros::spin-once)
 )
