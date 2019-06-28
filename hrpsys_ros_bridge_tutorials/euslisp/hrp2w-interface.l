(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2w "package://hrpsys_ros_bridge_tutorials/models/hrp2w.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-utils.l"))
  (require :hrp2w-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-utils.l"))
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "nav_msgs")

(defclass hrp2w-interface
  :super rtm-ros-robot-interface
  :slots (move-base-action move-base-trajectory-action
			   move-base-goal-msg move-base-goal-coords  move-base-goal-map-to-frame
			   base-frame-id
			   odom-topic move-base-trajectory-joint-names
			   go-pos-unsafe-goal-msg
			   current-goal-coords))
;; interface for simple base actions
;; partially copied from https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/master/pr2eus/robot-interface.l
(defmethod hrp2w-interface
  (:init (&rest args &key
		(move-base-action-name "move_base")
		((:base-frame-id base-frame-id-name) "/base_link")
		(base-controller-action-name "/base_controller/follow_joint_trajectory")
		(base-controller-joint-names (list "base_link_x" "base_link_y" "base_link_pan"))
		((:odom-topic odom-topic-name) "/odom_vmax") &allow-other-keys)
	 (prog1
	     (send-super* :init :robot hrp2w-robot args)
	   ;; add controller
	   (when (find-method self :def-limb-controller-method)
	     (dolist (limb '(:rarm :larm :head :torso))
	       (send self :def-limb-controller-method limb)
	       (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions t)))

	   ;;
	   (setq base-frame-id base-frame-id-name)
	   (setq odom-topic odom-topic-name)
	   (setq move-base-action (instance ros::simple-action-client :init
					    move-base-action-name move_base_msgs::MoveBaseAction
					    :groupname groupname))
	   (when base-controller-action-name
	     (setq move-base-trajectory-action
		   (instance ros::simple-action-client :init
			     base-controller-action-name
			     control_msgs::FollowJointTrajectoryAction
			     :groupname groupname))
	     (unless (send move-base-trajectory-action :wait-for-server 3)
	       (ros::ros-warn "move-base-trajectory-action is not found")
	       (setq move-base-trajectory-action nil))
	     (setq move-base-trajectory-joint-names base-controller-joint-names))
	   (ros::subscribe odom-topic-name nav_msgs::Odometry
			   #'send self :odom-callback :groupname groupname)
	   ))
  ;;
  (:odom-callback
   (msg)
   (let ((parsed
	  (list
	   (cons :stamp (send msg :header :stamp))
	   (cons :pose (ros::tf-pose->coords (send msg :pose :pose)))
	   (cons :velocity (float-vector
			    (* 1000 (send msg :twist :twist :linear :x))
			    (* 1000 (send msg :twist :twist :linear :y))
			    (send msg :twist :twist :angular :z))))))
     (send self :set-robot-state1 :odom parsed)))
  ;;
  (:go-stop (&optional (force-stop t))
	    (when joint-action-enable
	      (send move-base-action :cancel-all-goals)
	      (prog1
		  (send move-base-trajectory-action :cancel-all-goals)
		(if force-stop (send self :go-velocity 0 0 0)))
	      ))

  (:make-plan
   (st-cds goal-cds &key (start-frame-id "world") (goal-frame-id "world"))
   (let ((req (instance nav_msgs::GetPlanRequest :init))
	 (tm (ros::time-now))
	 map-to-frame
	 map-to-base
	 res
	 plan-cds-seq)
     (setq map-to-base (send *tfl* :lookup-transform "map" base-frame-id (ros::time 0)))
     (setq map-to-frame (send *tfl* :lookup-transform "map" start-frame-id (ros::time 0)))
     (send req :start :header :stamp)
     (send req :start :header :stamp tm)
     (if map-to-frame
	 (progn
	   (send req :start :header :frame_id "map")
	   (send req :start :pose (ros::coords->tf-pose (send (send st-cds :copy-worldcoords) :transform map-to-frame :world))))
       (progn
	 (send req :start :header :frame_id frame-id)
	 (send req :start :pose (ros::coords->tf-pose st-cds))))
     (setq map-to-frame (send *tfl* :lookup-transform "map" goal-frame-id (ros::time 0)))
     (send req :goal :header :stamp tm)
     (if map-to-frame
	 (progn
	   (send req :goal :header :frame_id "map")
	   (send req :goal :pose (ros::coords->tf-pose (send (send goal-cds :copy-worldcoords) :transform map-to-frame :world))))
       (progn
	 (send req :start :header :frame_id frame-id)
	 (send req :start :pose (ros::coords->tf-pose st-cds))))

     (setq res (ros::service-call "/move_base_node/make_plan" req))
     (unless (send res :plan :poses)
       (return-from :make-plan nil))
     (setq plan-cds-seq (mapcar #'(lambda (p-stamped)
				    (let ((cds (ros::tf-pose->coords (send p-stamped :pose))))
				      (if map-to-base
					  (progn
					    (send cds :transform (send map-to-base :inverse-transformation) :world)
					    ;;(send cds :transform (send *pr2* :copy-worldcoords) :world))
					    )
					cds)))
				(send res :plan :poses)))
     plan-cds-seq))
  ;;
  ;;
  (:move-to
   (coords &rest args &key (no-wait nil) &allow-other-keys)
   (send* self :move-to-send coords args)
   (if (not no-wait)
       (send* self :move-to-wait args)))
  (:move-to-send
   (coords &key (frame-id "world") (wait-for-server-timeout 5) (count 0) &allow-other-keys)
   (setq move-base-goal-msg (instance move_base_msgs::MoveBaseActionGoal :init))
   (setq move-base-goal-coords coords)
   (if (send self :simulation-modep)
       (let ()
         (cond ((equal frame-id base-frame-id)
                (setq current-goal-coords (send coords :transform robot :world))) ;; for simulation-callback
               (t
                (setq current-goal-coords (send coords :copy-worldcoords)))) ;; for simulation-callback
         (return-from :move-to-send)))
   (let (ret (count 0) (tm (ros::time-now))
	     (map-to-frame (send *tfl* :lookup-transform "map" frame-id (ros::time 0))))
     ;; store in slot variable for :move-to-wait
     (setq move-base-goal-map-to-frame map-to-frame)
     ;;
     (when (not (send move-base-action :wait-for-server wait-for-server-timeout))
       (return-from :move-to-send))
     ;;
     (send move-base-goal-msg :header :stamp tm)
     (send move-base-goal-msg :goal :target_pose :header :stamp tm)
     (if map-to-frame
	 (progn
	   (send move-base-goal-msg :goal :target_pose :header :frame_id "map")
	   (send move-base-goal-msg :goal :target_pose :pose
		 (ros::coords->tf-pose (send (send coords :copy-worldcoords) :transform map-to-frame :world))))
       (progn ;; fail to find "/map" to frame_id
	 (send move-base-goal-msg :goal :target_pose :header :frame_id frame-id)
	 (send move-base-goal-msg :goal :target_pose :pose (ros::coords->tf-pose coords))
	 ))
     (send move-base-goal-msg :header :seq count)
     (send move-base-goal-msg :goal :target_pose :header :seq count)
     (ros::ros-info "move-to : send-goal to ~A at ~A (~d)" (ros::tf-point->pos (send move-base-goal-msg :goal :target_pose :pose :position)) (send move-base-goal-msg :goal :target_pose :header :frame_id) count)
     (send move-base-action :send-goal move-base-goal-msg)
     move-base-goal-msg))
  (:move-to-wait
   (&rest args &key (retry 10) (frame-id "world") &allow-other-keys)
   (let (ret (count 0) (tm (ros::time-now))
	     (map-to-frame move-base-goal-map-to-frame)
             (coords move-base-goal-coords))
     (when (send self :simulation-modep)
       ;; wait for-result
       (while current-goal-coords
	 (send self :robot-interface-simulation-callback))
       (return-from :move-to-wait t)) ;; simlation-modep
     (if (null move-base-goal-msg) (return-from :move-to-wait nil))
     (while (and (null ret) (<= count retry))
       (when (> count 0) ;; retry
         (clear-costmap :node-name (send move-base-action :name))
         (send move-base-goal-msg :header :seq count)
         (send move-base-goal-msg :goal :target_pose :header :seq count)
         (ros::ros-info "move-to : send-goal to ~A at ~A (~d)" (ros::tf-point->pos (send move-base-goal-msg :goal :target_pose :pose :position)) (send move-base-goal-msg :goal :target_pose :header :frame_id) count)
         (send move-base-action :send-goal move-base-goal-msg))
       (send move-base-action :wait-for-result)
       (when (eq (send move-base-action :get-state)
                 actionlib_msgs::GoalStatus::*preempted*)
         (setq ret nil)
         (return))
       (if (eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
           (setq ret t))
       (incf count))
     (ros::ros-info "move-to : ~A" (if ret 'succeeded 'failed))
     ;;
     (when ret
       (let (diff diff-len current-coords lret map-goal-coords)
         ;;
         (setq map-goal-coords
               (if (string= frame-id base-frame-id)
                   (send (send map-to-frame :copy-worldcoords) :transform (send coords :worldcoords))
                 (send (send *tfl* :lookup-transform "map" frame-id (ros::time 0))
                       :transform (send coords :copy-worldcoords)))) ;; goal-coords in /map coordinates
         (setq lret (send *tfl* :wait-for-transform "map" base-frame-id (ros::time-now) 5))
         (ros::ros-warn ":move-to wait-for transform map to ~A -> ~A" base-frame-id lret)
         (when (null lret)
           (ros::ros-error ":move-to wait-for transform map to ~A failed" base-frame-id)
           (setq move-base-goal-msg nil)
           (return-from :move-to-no-wait nil))
         (setq current-coords (send *tfl* :lookup-transform "map" base-frame-id (ros::time 0)))
         (setq diff (send current-coords :transformation map-goal-coords))
         (ros::ros-warn ":move-to current-coords  ~A" current-coords)
         (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
         (ros::ros-warn "          error-coords   ~A" diff)
         (ros::ros-warn "         target-coords   ~A" coords)
         ;;
         (dotimes (i 2)
           (if (< (setq diff-len (norm (subseq (send diff :worldpos) 0 2))) 200) ;; move_base thre = 200mm
               (let* ((msec (* diff-len 10))
                      (x (/ (elt (send diff :worldpos) 0) msec))
                      (y (/ (elt (send diff :worldpos) 1) msec))
                      (d (/ (elt (car (rpy-angle (send diff :worldrot))) 0) (/ msec 1000))))
                 (ros::ros-warn ":move-to -> :go-velocity x:~A y:~A d:~A msec:~A" x y d msec)
                 (unix:usleep (* 400 1000)) ;; 400ms ???
                 (let ((acret (send self :go-velocity x y d msec :wait t)))
                   (unless acret
                     (setq move-base-goal-msg nil)
                     (return-from :move-to-wait nil)))
                 ;;(unix::usleep (* (round msec) 1000)) ;; why time wait
                 )
             (progn
               (ros::ros-error "too far from goal position ~A mm (> 200mm)" diff-len)
	       ;; move-to suceeded buf away from 200 mm
	       (ros::ros-error ":move-to try to send /move_base_simple/goal")
	       (ros::advertise "/move_base_simple/goal" geometry_msgs::PoseStamped 1)
	       (send move-base-goal-msg :goal :target_pose :header :seq (1+ count))
	       (ros::publish "/move_base_simple/goal" (send move-base-goal-msg :goal :target_pose))
	       (unix:sleep 3)
               (setq move-base-goal-msg nil)
               (return-from :move-to-wait nil)
               ))
           ;;
           (setq map-goal-coords
                 (if (string= frame-id base-frame-id)
                     (send (send map-to-frame :copy-worldcoords) :transform (send coords :worldcoords))
                   (send (send *tfl* :lookup-transform "map" frame-id (ros::time 0))
                         :transform (send coords :copy-worldcoords)))) ;; goal-coords in /map coordinates
           (setq lret (send *tfl* :wait-for-transform "map" base-frame-id (ros::time-now) 5))
           (ros::ros-warn ":move-to wait-for transform map to ~A -> ~A" base-frame-id lret)
           (when (null lret)
             (ros::ros-error ":move-to wait-for transform map to ~A failed" base-frame-id)
             (setq move-base-goal-msg nil)
             (return-from :move-to-wait nil))
           (setq current-coords (send *tfl* :lookup-transform "map" base-frame-id (ros::time 0)))
           (setq diff (send current-coords :transformation map-goal-coords))
           (ros::ros-warn ":move-to current-coords  ~A" current-coords)
           (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
           (ros::ros-warn "          error-coords   ~A" diff)
           (ros::ros-warn "         target-coords   ~A" coords)
           ) ;;  (do (i 2)
         ))
     (setq move-base-goal-msg nil) ;; :move-to-wait has been called
     ret))

  (:go-waitp
   ()
   (send self :spin-once) ;; update
   (when (send self :simulation-modep)
     (return-from :go-waitp nil)) ;; temporary
   (cond
    ((or
      (equal (send move-base-action :get-state) ros::*simple-goal-state-active*)
      (equal (send move-base-trajectory-action :get-state) ros::*simple-goal-state-active*))
     (return-from :go-waitp t))
    (t
     (return-from :go-waitp nil))))

  (:go-pos
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (let (c)
     (setq c (make-coords :pos (float-vector (* 1000 x) (* y 1000) 0)
			  :rpy (float-vector (deg2rad d) 0 0)))
     (send self :move-to c :retry 1 :frame-id base-frame-id)
     ))
  (:go-pos-no-wait
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (let (c)
     (setq c (make-coords :pos (float-vector (* 1000 x) (* y 1000) 0)
			  :rpy (float-vector (deg2rad d) 0 0)))
     (send self :move-to c :retry 1 :frame-id base-frame-id :no-wait t)
     ))
  (:go-wait
   ()
   (let ()
     (send self :move-to-wait :retry 1 :frame-id base-frame-id :no-wait nil)
     ))
  (:go-velocity
   (x y d ;; [m/sec] [m/sec] [rad/sec]
      &optional (msec 1000) ;; msec is total animation time [msec]
      &key (stop t) (wait))
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
	   (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x msec) (* y msec) 0)) ;; [m/sec] * [msec] = [mm]
       (send coords :rotate (deg2rad (/ (* d msec) 1000.0)) :z) ;; [rad/sec] * [msec] / 1000 = [rad]->(deg2rad)->[deg]
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	   ((> curr-tm msec))
	 (send robot :newcoords (midcoords (/ curr-tm (float msec)) orig-coords coords))
	 (if viewer (send self :draw-objects))))
     (return-from :go-velocity t))
   (unless move-base-trajectory-action
     (ros::ros-warn ":go-velocity is disabled. (move-base-trajectory-action is not found)")
     (return-from :go-velocity t))
   (setq x (* x (/ msec 1000.0))
         y (* y (/ msec 1000.0))
         d (* d (/ msec 1000.0)))
   (let ((goal (send self :move-trajectory x y d msec :stop stop)))
     (prog1
         (send move-base-trajectory-action :send-goal goal)
       (if wait (send move-base-trajectory-action :wait-for-result)))
     ))
  (:go-pos-unsafe (&rest args)
		  (send* self :go-pos-unsafe-no-wait args)
		  (send self :go-pos-unsafe-wait))
  (:go-pos-unsafe-no-wait
   (x y &optional (d 0) &key (speed-gain-vel 0.4) (speed-gain-rad 0.8)) ;; [m] [m] [degree]
   (ros::ros-info "go-pos-unsafe (x y d) = (~A ~A ~A)" x y d)
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
           (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x 1000) (* y 1000) 0))
       (send coords :rotate (deg2rad d) :z)
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
           ((> curr-tm 1000))
         (send robot :newcoords (midcoords (/ curr-tm 1000.0) orig-coords coords))
         (if viewer (send self :draw-objects))))
     (return-from :go-pos-unsafe-no-wait t))
   (unless move-base-trajectory-action
     (ros::ros-warn ":go-pose-unsafe-no-wait is disabled. (move-base-trajectory-action is not found)")
     (return-from :go-pos-unsafe-no-wait t))
   (let ((maxvel (* speed-gain-vel 0.295)) (maxrad (* speed-gain-rad 0.495))
         msec)
     ;; package://pr2_base_trajectory_action/config/pr2_base_link.yaml
     ;; 80% of maxvel = 0.3[m/sec]
     ;; 80% of maxrad = 0.5[rad/sec]
     (setq msec (* 1000 (max (/ (norm (float-vector x y)) (* maxvel 0.8))
                             (/ (abs (deg2rad d)) (* maxrad 0.8))
                             1.0)))
     (setq go-pos-unsafe-goal-msg (send self :move-trajectory
                                        x y (deg2rad d) msec
                                        :stop t))
     (send move-base-trajectory-action :send-goal go-pos-unsafe-goal-msg)
     ))
  (:go-pos-unsafe-wait ()
		       (let (x y d msec step goal (maxvel 0.295) (maxrad 0.495) (counter 0))
			 (if (null go-pos-unsafe-goal-msg) (return-from :go-pos-unsafe-wait nil))
			 (unless move-base-trajectory-action
			   (ros::ros-warn ":go-pose-unsafe-wait is disabled. (move-base-trajectory-action is not found)")
			   (return-from :go-pos-unsafe-wait t))
			 (while (< counter 3) ;; magic number 3 times
			   (let ((acret
				  (send move-base-trajectory-action :wait-for-result)))
			     (unless acret
			       (return-from :go-pos-unsafe-wait nil))
			     (send move-base-trajectory-action :spin-once))
			   (send self :spin-once)
			   (send self :spin-once)
			   (let ((goal-position (send (elt (send go-pos-unsafe-goal-msg :goal :trajectory :points) 1) :positions)) ;; goal
				 (odom-pos (scale 0.001 (send (send self :state :odom :pose) :pos)))
				 (odom-angle (elt (car (send (send self :state :odom :pose) :rpy-angle)) 0))
				 diff-position v)
			     (setq diff-position (v- goal-position (v+ odom-pos (float-vector 0 0 odom-angle))))
			     (setq v (v- (rotate-vector (float-vector (elt diff-position 0) (elt diff-position 1) 0)
							(- odom-angle) :z)
					 (float-vector 0 0 odom-angle)))
			     (setq x (elt v 0)
				   y (elt v 1)
				   d (rad2deg (elt diff-position 2)))
			     (if (> d (* 360 0.8)) (setq d (- d 360)))
			     (if (< d (* -360 0.8)) (setq d (+ d 360)))
			     (setq msec (* 1000 (max (/ (norm (float-vector x y)) (* maxvel 0.8))
						     (/ (abs (deg2rad d)) (* maxrad 0.8)))))
			     (setq msec (max msec 1000))
			     (setq step (/ 1000.0 msec))
			     (ros::ros-info "                diff-pos ~A ~A, diff-angle ~A" x y d)

			     (if (and (<= (sqrt (+ (* x x) (* y y)))  0.025)
				      (<= (abs d) 2.5) ;; 3 cm and 3deg
				      (/= counter 0))  ;; try at least 1 time
				 (progn
				   (setq go-pos-unsafe-goal-msg nil) ;; go-pos-unsafe-wait has been called
				   (return-from :go-pos-unsafe-wait t))
			       )
			     (setq go-pos-unsafe-goal-msg (send self :move-trajectory (* x step) (* y step)
								(* (deg2rad d) step) msec :stop t))
			     (send move-base-trajectory-action :send-goal go-pos-unsafe-goal-msg)
			     ) ;; let
			   (incf counter)
			   ) ;; while
			 (setq go-pos-unsafe-goal-msg nil) ;; go-pos-unsafe-wait has been called
			 t))
  ;;
  (:move-trajectory-sequence
   (trajectory-points time-list &key (stop t) (start-time) (send-action nil))
   "Move base following the trajectory points at each time points
    trajectory-points [ list of #f(x y d) ([m] for x, y; [rad] for d) ]
    time-list [list of time span [msec] ]
    stop [ stop after msec moveing ]
    start-time [ robot will move at start-time [sec or ros::Time] ]
    send-action [ send message to action server, it means robot will move ]"
   (send self :spin-once)
   (labels ((normalize-angle-positive (d)
				      (mod (+ (mod d 2pi) 2pi) 2pi))
            (normalize-angle (d)
			     (let ((a (normalize-angle-positive d)))
			       (if (> a pi) (- a 2pi) a)))
            (shortest-angular-distance (from to)
				       (normalize-angle (- to from))))
     (let ((odom-cds (send self :state :odom :pose))
           (msg (instance trajectory_msgs::JointTrajectory :init))
           (goal (instance control_msgs::FollowJointTrajectoryActionGoal :init))
           (cur-time 0) (nxt-time 0)
           cds-lst pts-msg-lst
           cur-cds nxt-cds)
       (send msg :joint_names move-base-trajectory-joint-names)
       ;; parse start-time
       (cond
	((numberp start-time)
	 (send msg :header :stamp (ros::time+ (ros::time-now) (ros::time start-time))))
	(start-time (send msg :header :stamp start-time))
	(t (send msg :header :stamp (ros::time-now))))
       (setq cds-lst
             (mapcar #'(lambda (pt)
                         (let ((cds (make-cascoords :coords odom-cds)))
                           (send cds :translate (float-vector
                                                 (* (elt pt 0) 1000.0)
                                                 (* (elt pt 1) 1000.0) 0))
                           (send cds :rotate (elt pt 2) :z)
                           cds))
                     trajectory-points))
       (setq cur-cds odom-cds
             nxt-cds (pop cds-lst)
             nxt-time (pop time-list)
             cur-yaw (caar (send cur-cds :rpy-angle))
             nxt-yaw 0)
       (while nxt-cds
         (unless (ros::ok) (return-from :move-trajectory-sequence nil))
         (let* ((tra  (send cur-cds :transformation nxt-cds))
                (rot (send cur-cds :rotate-vector (send tra :pos)))
                (diff-yaw (shortest-angular-distance
                           (caar (send cur-cds :rpy-angle))
                           (caar (send nxt-cds :rpy-angle)))))
           (push
            (instance trajectory_msgs::JointTrajectoryPoint :init
                      :positions  (float-vector
                                   (/ (elt (send cur-cds :pos) 0) 1000)
                                   (/ (elt (send cur-cds :pos) 1) 1000)
                                   cur-yaw)
                      :velocities (float-vector
                                   (/ (elt rot 0) nxt-time)
                                   (/ (elt rot 1) nxt-time)
                                   (/ (caar (send tra :rpy-angle)) nxt-time 0.001))
                      :time_from_start (ros::time cur-time))
            pts-msg-lst)

           (incf cur-time (/ nxt-time 1000.0))
           (setq cur-cds nxt-cds
                 nxt-cds (pop cds-lst)
                 nxt-time (pop time-list)
                 cur-yaw (if (> (caar (send tra :rpy-angle)) 0)
                             (+ cur-yaw (abs diff-yaw))
			   (- cur-yaw (abs diff-yaw)))))) ;; while
       (push ;; the last point
        (instance trajectory_msgs::JointTrajectoryPoint :init
                  :positions  (float-vector
                               (/ (elt (send cur-cds :pos) 0) 1000)
                               (/ (elt (send cur-cds :pos) 1) 1000)
                               cur-yaw)
                  :velocities (if stop
                                  (float-vector 0 0 0)
				(send (car pts-msg-lst) :velocities))
                  :time_from_start (ros::time cur-time))
        pts-msg-lst)
       (send msg :points (reverse pts-msg-lst))
       (send goal :goal :trajectory msg)
       (when send-action
         (unless move-base-trajectory-action
           (ros::ros-error "send-action is t, but move-base-trajectory-action is not found")
           (return-from :move-trajectory-sequence nil))
         (send move-base-trajectory-action :send-goal goal)
         (if (send move-base-trajectory-action :wait-for-result)
             (return-from :move-trajectory-sequence
               (send move-base-trajectory-action :get-result))
	   (return-from :move-trajectory-sequence nil)))
       goal)))
  ;;
  (:move-trajectory
   (x y d &optional (msec 1000) &key (stop t) (start-time) (send-action nil))
   "x [m] y [m] d [rad] msec [milli second (default: 1000)]
    stop [ stop after the base reached the goal if T (default: T)]
    start-time [ robot starts to move from `start-time` (default: now) ]
    send-action [ send the goal to action server if enabled, otherwise just returns goal without sending (default: nil) ]"
   (send self :move-trajectory-sequence
         (list (float-vector x y d))
         (list msec)
         :stop stop :start-time start-time :send-action send-action))
  ;;
  (:state
   (&rest args)
   (prog1
       (send-super* :state args)
     (case (car args)
       (:worldcoords
	(unless joint-action-enable
	  (return-from :state (send self :worldcoords)))
	(return-from :state (send *tfl* :lookup-transform (or (cadr args) "map") base-frame-id (ros::time)))))))
  ;;
  (:robot-interface-simulation-callback
   ()
   (when current-goal-coords
     (let* ((orig-coords (send robot :copy-worldcoords))
            (diff-pos (send orig-coords :difference-position current-goal-coords))
            (diff-rot (send orig-coords :difference-rotation current-goal-coords :rotation-axis :xy)))
       (cond
	((and (eps= (norm diff-pos) 0) (eps= (norm diff-rot) 0))
	 (setq current-goal-coords nil))
	(t
	 (send robot :newcoords (midcoords (min (/ 10 (max (norm diff-pos) 10))
						(/ 0.02 (max (norm diff-rot) 0.02)))
					   orig-coords current-goal-coords))))
       )) ;; when
   (send-super :robot-interface-simulation-callback)
   )
  )
;; hrp2w-interface (simple base actions)

;; methods for grasp controller
(defmethod hrp2w-interface
  (:get-hand-config-list () (list (list :larm "LHAND" "LARM_JOINT7") (list :rarm "RHAND" "RARM_JOINT7")))
  (:start-grasp
   (limb &key (target-error 0.15) (gain-percentage 5))
   (send-super :start-grasp limb :target-error target-error :gain-percentage gain-percentage)
   )
  )

(defun hrp2w-init (&key (sim nil) &rest args &allow-other-keys)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2w-interface :init args)))
  (if (not (boundp '*hrp2w*))
      (setq *hrp2w* (instance hrp2w-robot :init)))
  (unless sim
    (if (not (boundp '*vmax*))
	(progn
	  (require :vmax "package://vmax_controller/euslisp/vmax-interface.l")
	  (setq *vmax* (instance vmax-interface :init)))))
  (setq ((send *hrp2w* :head :neck-p) . max-angle) 44)
  t)
