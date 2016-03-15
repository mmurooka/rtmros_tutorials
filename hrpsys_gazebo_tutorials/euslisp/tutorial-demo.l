(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")

(ros::load-ros-manifest "jsk_recognition_msgs")

(ros::roseus "demo")

(defun setup
  ()
  ;; robot
  (hrp2jsk-init)
  (setq *robot* *hrp2jsk*)
  (objects (list *robot*))
  ;; ros topic
  (setq *topic-name* "/HSI_color_filter/boxes")
  (setq *bbox-coords* nil)
  (ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bbox-array-cb 1)
  (setq *tfl* (instance ros::transform-listener :init))
  )

(defun send-look-pose
  ()
  (send *robot* :reset-pose)
  (send *robot* :head :neck-p :joint-angle 35)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun bbox-array-cb (msg)
  (let* ((recog-coords (ros::tf-pose->coords (send (car (send msg :boxes)) :pose)))
         (trans-coords (send *tfl* :lookup-transform "rleg_end_coords" (send msg :header :frame_id) (ros::time 0))))
    (setq *bbox-coords* (send trans-coords :transform recog-coords))
    ))

(defun wait-until-new-bbox
  ()
  (setq *bbox-coords* nil)
  (ros::rate 10)
  (while (not *bbox-coords*)
    (ros::spin-once)
    (ros::sleep)
    )
  (warn ";; *bbox-coords* is ~a~%" *bbox-coords*)
  )

(defun go-pos-to-bbox
  ()
  (send *ri* :go-pos (- (* 1e-3 (elt (send *bbox-coords* :pos) 0)) 0.25) (* 1e-3 (elt (send *bbox-coords* :pos) 1)) 0)
  )

(defun send-reach-pose
  ()
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :fullbody-inverse-kinematics
        (list (make-coords :pos (float-vector 500 -100 200))
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              (send (send *robot* :rleg :end-coords) :copy-worldcoords))
        :move-target (mapcar #'(lambda (limb) (send *robot* limb :end-coords)) (list :rarm :lleg :rleg))
        :link-list (mapcar #'(lambda (limb) (send *robot* :link-list (send (send *robot* limb :end-coords) :parent))) (list :rarm :lleg :rleg))
        :rotation-axis (list nil t t)
        :debug-view :no-message
        )
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun demo
  ()
  (setup)
  (warn "Press Enter to start AutoBalancer~%")
  (send *ri* :start-auto-balancer)
  (warn "Press Enter to send look pose~%")
  (read-line)
  (send-look-pose)
  (warn "Press Enter to wait until new bbox~%")
  (read-line)
  (wait-until-new-bbox)
  (warn "Press Enter to go pos to object~%")
  (read-line)
  (send *bbox-coords* :draw-on :flush t :size 50 :width 5 :color #f(1 0 0))
  (go-pos-to-bbox)
  (warn "Press Enter to send reaching pose~%")
  (read-line)
  (send-reach-pose)
  )



(warn "(demo)~%")
