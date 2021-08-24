#!/usr/bin/env roseus
(ros::load-ros-manifest "memorization")
(ros::roseus "display_memory")

;; (setq points (instance geometry_msgs::PolygonStamped :init))


;; (ros::subscribe "~slave_larm_pose" geometry_msgs::PoseStamped #'call-back-l)

(defun call-back-params (msg)
  (format t "in cb-params/n")
  (setq boxes (send msg box))
  (setq radious (send msg radious))
  (setq objects-list
        (mapcar #'(lambda (box) (prong (setq sorted_list (quick-sort (list (send (send box dimensions) x) (send (send box dimensions) y) (send (send box dimensions) z)))) (make-cylinder (/ (+ (elt sorted_list 1) (elt sorted_list 2)) 4) (elt sorted_list 0)))) boxes))

  (objcts objects-list)
  )

(defun call-back-r (msg)
  (format t "in rarm-cb/n")
  )
(ros::subscribe "~slave_rarm_pose" geometry_msgs::PoseStamped #'call-back-r)
(ros::subscribe "~recognized_params" memorization::RecognizedParams #'call-back-params)
;; (ros::subscribe "memorize/recognizedparams" memorization::RecognizedParams #'call-back-params)
(do-until-key
 ;; (setq cnt (+ cnt 1))
 ;;平面がない時はリセット
 ;; (if (= cnt 10000000)
 ;;     (progn
 ;;        (if (equal tmp-point-list (send points :polygon :points))
 ;;            (progn
 ;;              (print "reset")
 ;;              (setq points (instance geometry_msgs::PolygonStamped :init))
 ;;              ))
 ;;        (setq tmp-point-list (send points :polygon :points))
 ;;        (setq cnt 0)))
  (ros::spin-once)
 )
