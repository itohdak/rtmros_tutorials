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
			   odom-topic
			   go-pos-unsafe-goal-msg
			   current-goal-coords))

(defmethod hrp2w-interface
  (:init (&rest args &key
		(move-base-action-name "move_base")
		((:base-frame-id base-frame-id-name) "/base_link")
		(base-controller-action-name "/base_controller/follow_joint_trajectory")
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
	       (setq move-base-trajectory-action nil)))
	   (ros::subscribe odom-topic-name nav_msgs::Odometry
			   #'send self :odom-callback :groupname groupname)
	   ))
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
   (let ((goal (send self :move-trajectory x y d msec :stop stop)))
     (prog1
         (send move-base-trajectory-action :send-goal goal)
       (if wait (send move-base-trajectory-action :wait-for-result)))
     ))
  (:go-pos-unsafe (&rest args)
		  (send* self :go-pos-unsafe-no-wait args)
		  (send self :go-pos-unsafe-wait))
  (:go-pos-unsafe-no-wait
   (x y &optional (d 0)) ;; [m] [m] [degree]
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
   (let (msec step (maxvel 0.295) (maxrad 0.495))
     ;; package://pr2_base_trajectory_action/config/pr2_base_link.yaml
     ;; 80% of maxvel = 0.3[m/sec]
     ;; 80% of maxrad = 0.5[rad/sec]
     (setq msec (* 1000 (max (/ (norm (float-vector x y)) (* maxvel 0.8))
                             (/ (abs (deg2rad d)) (* maxrad 0.8)))))
     (setq msec (max msec 1000))
     (setq step (/ 1000.0 msec))
     (setq go-pos-unsafe-goal-msg (send self :move-trajectory (* x step) (* y step)
                                        (* (deg2rad d) step) msec :stop t))
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
   "trajectory-points [ list of #f(x y d) ] time-list [list of time span]
stop [ stop after msec moveing ]
start-time [ robot will move at start-time ]
send-action [ send message to action server, it means robot will move ]"
   (send self :move-trajectory trajectory-points time-list nil nil
         :stop stop :start-time start-time :send-action send-action))
  (:move-trajectory
   (x y d &optional (msec 1000) &key (stop t) (start-time) (send-action nil))
   " x [m/sec] y [m/sec] d [rad/sec] msec [milli second]
stop [ stop after msec moveing ]
start-time [ robot will move at start-time ]
send-action [ send message to action server, it means robot will move ]"
   (send self :spin-once)
   (let ((odom-cds (send self :state :odom :pose))
         (msg (instance trajectory_msgs::JointTrajectory :init))
         (goal (instance control_msgs::FollowJointTrajectoryActionGoal :init)))
     (cond
      ((numberp start-time)
       (send msg :header :stamp (ros::time+ (ros::time-now) (ros::time start-time))))
      (start-time (send msg :header :stamp start-time))
      (t (send msg :header :stamp (ros::time-now))))
     (send msg :joint_names (list "base_link_x" "base_link_y" "base_link_pan"))
     (cond
      ((and (listp x) (listp y))
       (let ((traj-lst
              (mapcar #'(lambda (vec)
                          (send
                           (make-coords :pos (float-vector (elt vec 0) (elt vec 1) 0)
                                        :rpy (list (elt vec 2) 0 0))
                           :transform odom-cds :world)) x))
             (cur-cds (send odom-cds :copy-worldcoords))
             (cur-time 0)
             nxt-cds nxt-time ret)
         (pprint traj-lst)
         (setq nxt-cds  (pop traj-lst)
               nxt-time (pop y))
         (while nxt-cds
           (let* ((vr (send cur-cds :transformation nxt-cds))
                  (vp (send cur-cds :rotate-vector (send vr :pos))))
             (push
              (instance trajectory_msgs::JointTrajectoryPoint :init
                        :positions (float-vector
                                    (/ (elt (send cur-cds :pos) 0) 1000)
                                    (/ (elt (send cur-cds :pos) 1) 1000)
                                    (caar (send cur-cds :rpy-angle)))
                        :velocities (float-vector
                                     (/ (elt vp 0) nxt-time)
                                     (/ (elt vp 1) nxt-time)
                                     (/ (caar (send vr :rpy-angle)) nxt-time 0.001))
                        :time_from_start (ros::time cur-time))
              ret))
           (incf cur-time (/ nxt-time 1000.0))
           (setq cur-cds  nxt-cds
                 nxt-cds  (pop traj-lst)
                 nxt-time (pop y))
           ) ;; (while nxt-cds
         (push ;; push last point
          (instance trajectory_msgs::JointTrajectoryPoint :init
                    :positions (float-vector
                                (/ (elt (send cur-cds :pos) 0) 1000)
                                (/ (elt (send cur-cds :pos) 1) 1000)
                                (caar (send cur-cds :rpy-angle)))
                    :velocities (if stop (float-vector 0 0 0)
                                  (send (car ret) :velocities))
                    :time_from_start (ros::time cur-time))
          ret)
         (send msg :points (nreverse ret))
         ))
      (t
       (let ((sec (/ msec 1000.0))
             (odom-angle (elt (car (send odom-cds :rpy-angle)) 0))
             (odom-pos (scale 0.001 (send odom-cds :pos)))
             (pt1 (instance trajectory_msgs::JointTrajectoryPoint :init))
             (pt2 (instance trajectory_msgs::JointTrajectoryPoint :init)))
         (send pt1 :time_from_start (ros::time))
         (send pt2 :time_from_start (ros::time sec))
         (send pt1 :positions (v+ odom-pos
                                  (float-vector 0 0 odom-angle)))
         (send pt2 :positions (v+ (v+ odom-pos
                                      (float-vector 0 0 (+ odom-angle (* sec d))))
                                  (rotate-vector (scale sec (float-vector x y 0))
                                                 odom-angle :z)))
         (send pt1 :velocities (rotate-vector (float-vector x y d) odom-angle :z))
         (if stop
             (send pt2 :velocities (float-vector 0 0 0)) ;; To stop just
           (send pt2 :velocities (rotate-vector (float-vector x y d) odom-angle :z)))

         (ros::ros-debug "move-trajectory (x y d msec) = (~A ~A ~A ~A)" (* sec x) (* sec y) (* sec d) msec)
         (ros::ros-debug "                odom-pos ~A, odom-angle ~A" odom-pos odom-angle)
         (ros::ros-debug "                pt1 ~A" (send pt1 :positions))
         (ros::ros-debug "                pt2 ~A" (send pt2 :positions))
         ;;
         (send msg :points (list pt1 pt2)))
       ))
     (send goal :goal :trajectory msg)
     (when (and send-action move-base-trajectory-action)
       (send move-base-trajectory-action :send-goal goal)
       (let ((acret
              (send move-base-trajectory-action :wait-for-result)))
         (unless acret (return-from :move-trajectory nil))
         (send move-base-trajectory-action :spin-once)))
     ;;
     goal))
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

;; partially copied from https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_fetch_robot/fetcheus/fetch-interface.l
;; interface for simple base actions
;; (defmethod hrp2w-interface
;;   (:send-cmd-vel-raw (x d &key (topic-name "/move_base/cmd_vel")) ;; "/cmd_vel"
;; 		     (when (send self :simulation-modep)
;; 		       (return-from :send-cmd-vel-raw t))
;; 		     (unless (ros::get-topic-publisher topic-name)
;; 		       (ros::advertise topic-name geometry_msgs::Twist 1)
;; 		       (unix:sleep 1))
;; 		     (let ((msg (instance geometry_msgs::Twist :init)))
;; 		       (send msg :linear :x x)
;; 		       (send msg :angular :z d)
;; 		       (ros::publish topic-name msg)))
;;   (:go-velocity
;;    (x y d ;; [m/sec] [m/sec] [rad/sec]
;;       &optional (msec 1000) ;; msec is total animation time [msec]
;;       &key (stop t) (wait t))
;;    (unless (not (zerop y))
;;      (ros::ros-error
;;       "Setting velocity in y-axis is not valid for non-holonomic robot")
;;      (return-from :go-velocity nil))
;;    (unless wait
;;      (ros::ros-error ":go-velocity without wait is unsupported")
;;      (return-from :go-velocity nil))
;;    (setq msec (- msec (ros::get-param "/base_controller/timeout" 0.25)))
;;    (ros::rate 100)
;;    (let ((start-time (ros::time-now)))
;;      (while (and (ros::ok)
;; 		 (< (* 1000.0 (send (ros::time- (ros::time-now) start-time) :to-sec)) msec))
;;        (send self :spin-once)
;;        (send self :send-cmd-vel-raw x d)
;;        (ros::sleep)))
;;    (when stop
;;      (send self :send-cmd-vel-raw 0 0))
;;    (ros::rate 10)
;;    t)
;;   (:go-pos-unsafe
;;    (&rest args)
;;    (send self :put :go-pos-unsafe-no-wait-goal (coerce args float-vector))
;;    (send self :go-pos-unsafe-wait)
;;    t)
;;   (:go-pos-unsafe-no-wait
;;    (x y &optional (d 0)) ;; [m] [m] [degree]
;;    (ros::ros-warn ":go-pos-unsafe-no-wait is not supported for this robot.")
;;    (send self :put :go-pos-unsafe-no-wait-goal (float-vector x y d))
;;    t)
;;   (:go-pos-unsafe-wait
;;    (&key (translation-threshold 0.05) (rotation-threshold (deg2rad 5))
;; 	 (translation-gain 0.3) (rotation-gain 0.3))
;;    (unless (send self :get :go-pos-unsafe-no-wait-goal)
;;      (ros::ros-error ":go-pos-unsafe-wait is called without goal")
;;      (return-from :go-pos-unsafe-wait nil))
;;    (ros::rate 100)
;;    (labels ((shortest-angle
;; 	     (d0 d1)
;; 	     (atan2 (sin (- d0 d1)) (cos (- d0 d1)))))
;;      (let* ((goal (send self :get :go-pos-unsafe-no-wait-goal))
;; 	    (x (aref goal 0))
;; 	    (y (aref goal 1))
;; 	    (d (aref goal 2))
;; 	    (forward-p (>= x 0))
;; 	    (d0 (if forward-p (atan2 y x) (atan2 (- y) (- x))))
;; 	    (d1 (distance (float-vector 0 0) (float-vector x y)))
;; 	    (d2 (shortest-angle (- d d0) 0))
;; 	    org-cds cur-cds diffrot diffpos err)
;;        (send self :remprop :go-pos-unsafe-no-wait-goal)

;;        ;; yaw-x-yaw approach
;;        (ros::ros-info "go-pos-unsafe: (yaw, x, yaw) = (~A ~A ~A)" d0 d1 d2)

;;        ;; yaw
;;        (send self :spin-once)
;;        (setq org-cds (send self :state :odom :pose))
;;        (ros::ros-debug "org-cds: ~A" org-cds)
;;        (while (ros::ok)
;; 	 (send self :spin-once)
;; 	 (setq cur-cds (send self :state :odom :pose))
;; 	 (setq diffrot (aref (send org-cds :difference-rotation cur-cds) 2))
;; 	 (setq err (shortest-angle d0 diffrot))
;; 	 (ros::ros-debug "diffrot: ~A, err: ~A" diffrot err)
;; 	 (when (< (abs err) rotation-threshold)
;; 	   (unless (send self :simulation-modep)
;; 	     (send self :send-cmd-vel-raw 0 0)) ;; stop
;; 	   (return))
;; 	 (when (send self :simulation-modep)
;; 	   (send self :robot :rotate (deg2rad (if (> err 0) 5 -5)) :z)
;; 	   (send self :draw-objects))
;; 	 (send self :send-cmd-vel-raw 0 (* rotation-gain err))
;; 	 (ros::sleep))

;;        ;; x
;;        (setq org-cds (send self :state :odom :pose))
;;        (while (ros::ok)
;; 	 (send self :spin-once)
;; 	 (setq cur-cds (send self :state :odom :pose))
;; 	 (setq diffpos (* 0.001 (norm (send org-cds :difference-position cur-cds))))
;; 	 (setq err (- d1 diffpos))
;; 	 (ros::ros-debug "diffpos: ~A, err: ~A" diffpos err)
;; 	 (when (< (abs err) translation-threshold)
;; 	   (send self :send-cmd-vel-raw 0 0) ;; stop
;; 	   (return))
;; 	 (when (send self :simulation-modep)
;; 	   (send self :robot :newcoords
;; 		 (send (send self :robot :copy-worldcoords)
;; 		       :translate (float-vector
;; 				   (if forward-p 0.1 -0.1) 0 0)))
;; 	   (send self :draw-objects))
;; 	 (send self :send-cmd-vel-raw
;; 	       (* translation-gain (if forward-p err (- err))) 0)
;; 	 (ros::sleep))

;;        ;; yaw
;;        (setq org-cds (send self :state :odom :pose))
;;        (while (ros::ok)
;; 	 (send self :spin-once)
;; 	 (setq cur-cds (send self :state :odom :pose))
;; 	 (setq diffrot (aref (send org-cds :difference-rotation cur-cds) 2))
;; 	 (setq err (shortest-angle d2 diffrot))
;; 	 (ros::ros-debug "diffrot: ~A, err: ~A" diffrot err)
;; 	 (when (< (abs err) rotation-threshold)
;; 	   (send self :send-cmd-vel-raw 0 0) ;; stop
;; 	   (return))
;; 	 (when (send self :simulation-modep)
;; 	   (send self :robot :rotate (deg2rad (if (> err 0) 5 -5)) :z)
;; 	   (send self :draw-objects))
;; 	 (send self :send-cmd-vel-raw 0 (* rotation-gain err))
;; 	 (ros::sleep))
;;        t)))
;;   )
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
