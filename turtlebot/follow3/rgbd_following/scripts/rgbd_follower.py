#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Live RGB-D trail -> rolling odom navigation, with explicit completion checks."""
import math
import threading
import time
import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rgbd_following_core.motion import fresh, need_goal, observed_prefix, clear_standoff
from rgbd_following_core.planning import reachable_standoff
from rgbd_following_core.settings import validate
from rgbd_following_core.tracking import Trail, Completion, distance, circle_inside_polygon
from rgbd_following_core.ros_helpers import same_frame, grid_message, transform_pose, OdometryState


class Follower(object):
    def __init__(self):
        self.c = validate(rospy.get_param('~settings'), rospy.get_param('~execute', False))
        self.frame, self.base = rospy.get_param('~odom_frame', 'odom'), rospy.get_param('~base_frame', 'base_footprint')
        self.lock = threading.RLock()
        self.clock = getattr(time, 'monotonic', time.time)
        self.trail = Trail()
        self.completion = Completion(self.c['finish_hold'], self.c['finish_motion_tolerance'])
        self.target = self.target_stamp = self.target_received = None
        self.valid = False
        self.valid_received = self.mission_received = None
        self.mission, self.estop = 'idle', False
        self.grid = self.grid_stamp = self.grid_received = None
        self.goal, self.goal_time = None, 0.
        self.lost_since = None
        self.failed = None
        self.finished = False
        self.previous_pose = None
        self.odometry = OdometryState()
        self.trail_interrupted = False
        self.rebuilding_trail = False
        self.goal_sequence = 0
        self.retry_pending = False
        self.retry_not_before = 0.
        self.retry_map_stamp = None
        self.failed_goal = None
        self.selection_cache = None
        self.excluded_goals = []
        self.goal_is_detour = False
        self.goal_is_alternate = False
        self.goal_target_anchor = None
        self.gate_reason = None
        self.gate_received = self.gate_last_limited = None
        self.gate_limited_seconds = 0.
        self.progress_anchor = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.allowed = rospy.Publisher('motion_allowed', Bool, queue_size=1)
        self.status = rospy.Publisher('follow_status', String, queue_size=1, latch=True)
        self.subs = [rospy.Subscriber('target_pose', PoseStamped, self.on_target, queue_size=1),
                     rospy.Subscriber('target_valid', Bool, self.on_valid, queue_size=1),
                     rospy.Subscriber('move_base/global_costmap/costmap', OccupancyGrid, self.on_grid, queue_size=1),
                     rospy.Subscriber('mission', String, self.on_mission, queue_size=1),
                     rospy.Subscriber('estop', Bool, self.on_estop, queue_size=1)]
        self.subs.append(rospy.Subscriber('odom', Odometry, self.on_odom, queue_size=1))
        self.subs.append(rospy.Subscriber('gate_status', String, self.on_gate, queue_size=1))
        rospy.on_shutdown(self.stop)
        self.timer = rospy.Timer(rospy.Duration(.1), self.tick)

    def on_target(self, msg):
        with self.lock:
            stamp = msg.header.stamp.to_sec()
            if (not same_frame(msg.header.frame_id, self.frame) or
                    not fresh(stamp, rospy.Time.now().to_sec(), self.c['target_timeout']) or
                    (self.target_stamp is not None and stamp <= self.target_stamp)):
                return
            point = (msg.pose.position.x, msg.pose.position.y)
            if self.trail_interrupted or (self.target_stamp is not None and stamp-self.target_stamp > self.c['target_timeout']):
                # No invented straight segment across an unobserved turn.
                self.trail = Trail()
                self.rebuilding_trail, self.trail_interrupted = True, False
            if not self.trail.add(point):
                self.failed = 'target_jump_reset_required'
                self.stop()
                return
            self.target, self.target_stamp, self.target_received = point, stamp, self.clock()

    def on_valid(self, msg):
        with self.lock:
            self.valid, self.valid_received = msg.data, self.clock()
            if not self.valid:
                self.trail_interrupted = self.target is not None
                self.stop()

    def on_odom(self, msg):
        with self.lock:
            self.odometry.update(msg, self.frame, self.base, rospy.Time.now().to_sec(), self.clock(), self.c['odom_timeout'])

    def on_grid(self, msg):
        with self.lock:
            try:
                self.grid = grid_message(msg, self.frame)
                self.grid_stamp, self.grid_received = msg.header.stamp.to_sec(), self.clock()
            except ValueError:
                self.grid = None

    def on_mission(self, msg):
        with self.lock:
            self.mission, self.mission_received = msg.data, self.clock()

    def on_estop(self, msg):
        with self.lock:
            self.estop = msg.data
            if self.estop:
                self.stop()

    def on_gate(self, msg):
        with self.lock:
            wall = self.clock()
            limited = ('visibility_or_clearance_slowdown', 'visibility_path_blocked',
                       'measured_visibility_stopping_path_blocked')
            previous = getattr(self, 'gate_received', None)
            if (fresh(previous, wall, .3) and getattr(self, 'gate_reason', None) in limited and
                    self.goal is not None and self.mission in ('carrying', 'releasing')):
                self.gate_limited_seconds = getattr(self, 'gate_limited_seconds', 0.)+wall-previous
            self.gate_reason, self.gate_received = msg.data, wall
            if msg.data in limited:
                self.gate_last_limited = wall

    def reset_progress(self):
        self.progress_anchor = None
        self.gate_limited_seconds = 0.
        self.gate_last_limited = None

    def stalled_by_gate(self, robot, wall, state):
        """Replan when scaled commands repeatedly fail to produce real motion.

        Small permitted velocities can fall below the base's static friction.
        Count limited intervals across interleaved healthy gate messages, but
        use fresh odometry/TF displacement and yaw to distinguish useful turns
        from a stalled command. A stopped robot already at its gap is expected.
        """
        if (self.goal is None or state not in (0, 1) or self.mission not in ('carrying', 'releasing') or
                self.c.get('navigation_retry_delay', 0.) <= 0 or distance(robot, self.goal) <= .10 or
                distance(robot, self.target) <= self.c['follow_distance']+.05):
            self.reset_progress()
            return False
        anchor = getattr(self, 'progress_anchor', None)
        if anchor is None:
            self.progress_anchor = robot
            self.gate_limited_seconds = 0.
            return False
        turn = abs(math.atan2(math.sin(robot[2]-anchor[2]), math.cos(robot[2]-anchor[2])))
        if distance(robot, anchor) >= .01 or turn >= .05:
            self.progress_anchor = robot
            self.gate_limited_seconds = 0.
            return False
        if not fresh(getattr(self, 'gate_last_limited', None), wall, 1.):
            self.gate_limited_seconds = 0.
            return False
        if (getattr(self, 'gate_limited_seconds', 0.) < 2. or
                not fresh(getattr(self, 'gate_received', None), wall, .3)):
            return False
        # Invalidate the old callback before cancel_goal; keep the normal retry
        # delay/new-map requirement and remember the unusable goal for ranking.
        self.goal_sequence = getattr(self, 'goal_sequence', 0)+1
        self.client.cancel_goal()
        self.navigation_failure(4)
        self.status.publish('navigation_retry_no_progress')
        return True

    def remember_goal(self, point):
        if point is not None:
            entries = getattr(self, 'excluded_goals', [])
            self.excluded_goals = (entries+[(point, self.clock(), self.target)])[-12:]
            self.selection_cache = None

    def stop(self):
        self.reset_progress()
        self.allowed.publish(False)
        if self.goal is not None:
            if self.client.get_state() in (4, 5, 9):
                self.navigation_failure(self.client.get_state())
            self.goal_sequence = getattr(self, 'goal_sequence', 0)+1
            self.client.cancel_goal()
            self.goal = None

    def navigation_failure(self, state):
        self.reset_progress()
        if state in (4, 5) and self.c.get('navigation_retry_delay', 0.) > 0 and self.mission != 'done':
            if not getattr(self, 'retry_pending', False):
                self.failed_goal = self.goal
                self.remember_goal(self.goal)
                self.retry_map_stamp = self.grid_stamp
                self.retry_not_before = self.clock()+self.c['navigation_retry_delay']
                self.retry_pending = True
                self.selection_cache = None
            self.goal = None
            self.goal_sequence = getattr(self, 'goal_sequence', 0)+1
        else:
            self.failed = 'navigation_failed_reset_required'
        self.allowed.publish(False)

    def on_navigation_done(self, state, result, sequence=None):
        with self.lock:
            if sequence is not None and sequence != getattr(self, 'goal_sequence', 0):
                return
            if state in (4, 5, 9):
                self.navigation_failure(state)

    def block(self, reason, reset_finish=True):
        self.stop()
        if reset_finish:
            self.completion.reset()
        self.status.publish(reason)

    def tick(self, event):
        with self.lock:
            try:
                self.step()
            except Exception as exc:
                self.failed = 'controller_exception_reset_required'
                self.block(self.failed)
                rospy.logerr_throttle(2, str(exc))

    def step(self):
        now, wall, c = rospy.Time.now().to_sec(), self.clock(), self.c
        # Observe failure before any transient block cancels and clears the goal.
        state = self.client.get_state() if self.goal is not None else None
        if state in (4, 5, 9):
            self.navigation_failure(state)
        if self.finished or self.failed:
            return self.block('finished' if self.finished else self.failed)
        if self.estop or self.mission not in ('carrying', 'releasing', 'done') or not fresh(self.mission_received, wall, c['mission_timeout']):
            self.lost_since = None
            return self.block('mission_stopped')
        if (not self.valid or not fresh(self.valid_received, wall, c['target_timeout']) or
                not fresh(self.target_received, wall, c['target_timeout']) or
                not fresh(self.target_stamp, now, c['target_timeout'])):
            if self.lost_since is None:
                self.lost_since = wall
            if wall - self.lost_since >= c['lost_failure_seconds']:
                self.failed = 'target_lost_over_limit_reset_required'
            return self.block(self.failed or 'target_lost_hold_position')
        self.lost_since = None
        if not self.odometry.valid(now, wall, c['odom_timeout']):
            return self.block('odometry_missing_or_stale')
        if (self.grid is None or not fresh(self.grid_received, wall, c['grid_timeout']) or
                not fresh(self.grid_stamp, now, c['grid_timeout'])):
            return self.block('costmap_missing_or_stale')
        if getattr(self, 'retry_pending', False):
            if wall < self.retry_not_before or self.grid_stamp <= self.retry_map_stamp:
                return self.block('navigation_retry_wait')
            self.retry_pending = False
        try:
            transform = self.buffer.lookup_transform(self.frame, self.base, rospy.Time(0), rospy.Duration(.03))
            if not fresh(transform.header.stamp.to_sec(), now, c['tf_timeout']):
                return self.block('base_tf_stale')
            robot = transform_pose(transform)
        except (ValueError, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return self.block('base_tf_invalid')
        if self.previous_pose is not None and distance(robot, self.previous_pose) > .5:
            self.failed = 'odom_jump_reset_required'
            return self.block(self.failed)
        self.previous_pose = robot
        if distance(robot, self.target) <= c['minimum_distance']:
            return self.block('target_too_close')
        if self.stalled_by_gate(robot, wall, state):
            return
        self.excluded_goals = [entry for entry in getattr(self, 'excluded_goals', [])
                               if wall-entry[1] < 10. and distance(self.target, entry[2]) < .3]
        target_bearing = math.atan2(self.target[1]-robot[1], self.target[0]-robot[0])
        facing_target = abs(math.atan2(math.sin(target_bearing-robot[2]),
                                     math.cos(target_bearing-robot[2]))) <= .25
        if (getattr(self, 'goal_is_detour', False) and self.goal is not None and
                (state == 3 or (distance(robot, self.goal) <= .07 and facing_target))):
            self.remember_goal(self.goal)
            self.stop()
        self.failed_goal = None if not self.excluded_goals else self.excluded_goals[-1][0]
        point = self.trail.behind(c['follow_distance'])
        live_standoff = False
        if point is not None:
            self.rebuilding_trail = False
        elif self.trail.points:
            # On initial acquisition as well as verified reacquisition, use a
            # live standoff goal until enough observed trail exists. This goal
            # is planned by move_base through observed free space; it does not
            # add an invented leader segment to the measured trail.
            live_standoff = True
            dx, dy = self.target[0]-robot[0], self.target[1]-robot[1]
            length = math.hypot(dx, dy)
            if length >= c['follow_distance']:
                point = (self.target[0]-dx/length*c['follow_distance'],
                         self.target[1]-dy/length*c['follow_distance'], math.atan2(dy, dx))
            elif self.mission == 'done' and length >= c['follow_distance']-c['finish_tolerance']:
                # Already within the final gap tolerance: hold and measure stop;
                # do not demand new leader motion or reverse to build a trail.
                point = (robot[0], robot[1], math.atan2(dy, dx))
        if point is None:
            return self.block('waiting_for_target_trail')
        if self.completion.ready(self.mission, self.target, wall, robot, point,
                                 c['finish_tolerance'], c['finish_polygon'], c['footprint_radius'],
                                 self.odometry.speed <= c['stopped_linear'] and self.odometry.angular <= c['stopped_angular']):
            self.finished = True
            return self.block('finished')
        partial_goal = False
        alternate_goal = False
        detour_goal = False
        excluded = [entry[0] for entry in self.excluded_goals]
        holding = (self.goal is not None and state in (0, 1) and self.mission != 'done' and
                   (getattr(self, 'goal_is_detour', False) or getattr(self, 'goal_is_alternate', False)) and
                   wall-self.goal_time < c.get('detour_goal_hold', 0.) and
                   getattr(self, 'goal_target_anchor', None) is not None and
                   distance(self.target, self.goal_target_anchor) < .15 and
                   self.grid.clear(self.goal[0], self.goal[1], c['footprint_radius']))
        if holding:
            # Keep a usable route long enough to move along it. Small depth
            # fluctuations must not alternate opposite-side goals every frame.
            # A moved target or a newly blocked endpoint invalidates this hold.
            point = self.goal
            detour_goal = self.goal_is_detour
            alternate_goal = getattr(self, 'goal_is_alternate', False)
        elif c.get('detour_max_extra_distance', 0.) > 0 and self.mission != 'done':
            # A clear destination says nothing about the approach. Use the
            # same reachable route selection for every field follow goal;
            # otherwise a cone between two clear endpoints bypasses the
            # visibility-aware intermediate-waypoint planner entirely.
            cache = getattr(self, 'selection_cache', None)
            if (cache is not None and wall-cache[0] < .5 and distance(robot, cache[1]) < .05 and
                    distance(self.target, cache[2]) < .05 and cache[3] is not None and
                    self.grid.clear(cache[3][0][0], cache[3][0][1], c['footprint_radius'])):
                selected = cache[3]
            else:
                selected = reachable_standoff(self.grid, robot, self.target, c['follow_distance'],
                    c.get('standoff_min_distance', c['follow_distance']), c['footprint_radius'],
                    c['detour_max_extra_distance'], excluded)
                self.selection_cache = (wall, robot, self.target, selected)
            if selected is None:
                return self.block('waiting_for_reachable_follow_goal')
            point, detour_goal = selected
            alternate_goal = not detour_goal
        if not self.grid.clear(point[0], point[1], c['footprint_radius']):
            alternate = (clear_standoff(self.grid, robot, self.target, c['follow_distance'],
                         c['footprint_radius'], c.get('standoff_min_distance', c['follow_distance']))
                         if live_standoff and self.mission != 'done' and
                         'standoff_min_distance' in c else None)
            if alternate is not None:
                point, alternate_goal = alternate, True
        if not self.grid.clear(point[0], point[1], c['footprint_radius']):
            prefix = (observed_prefix(self.grid, robot, point, c['footprint_radius'])
                      if live_standoff and self.mission != 'done' else None)
            if prefix is None:
                return self.block('follow_goal_blocked')
            point, partial_goal = prefix, True
        if self.mission == 'done' and distance(robot, point) <= c['finish_tolerance']:
            # Hold at final gap while verifying stationary target and finish zone.
            reason = 'finish_region_unconfigured'
            if c['finish_polygon']:
                reason = 'settling_at_finish' if circle_inside_polygon(robot, c['footprint_radius'], c['finish_polygon']) else 'finish_zone_not_reached'
            return self.block(reason, False)
        if not self.client.wait_for_server(rospy.Duration(.01)):
            return self.block('navigation_unavailable')
        if (need_goal(self.goal, point, state, self.mission == 'done', wall - self.goal_time) or
                (self.goal is not None and not self.grid.clear(self.goal[0], self.goal[1], c['footprint_radius']))):
            msg = MoveBaseGoal()
            msg.target_pose.header.frame_id, msg.target_pose.header.stamp = self.frame, rospy.Time.now()
            msg.target_pose.pose.position.x, msg.target_pose.pose.position.y = point[:2]
            msg.target_pose.pose.orientation.z = math.sin(point[2] / 2)
            msg.target_pose.pose.orientation.w = math.cos(point[2] / 2)
            self.goal_sequence = getattr(self, 'goal_sequence', 0)+1
            sequence = self.goal_sequence
            self.reset_progress()
            self.goal, self.goal_time = point, wall
            self.goal_is_detour = detour_goal
            self.goal_is_alternate = alternate_goal
            self.goal_target_anchor = self.target
            self.client.send_goal(msg, done_cb=lambda status, result: self.on_navigation_done(status, result, sequence))
        self.allowed.publish(True)
        self.status.publish('following_detour_waypoint' if detour_goal else
                            ('following_alternate_standoff' if alternate_goal else
                            ('following_observed_prefix' if partial_goal else
                            ('following_live_standoff' if live_standoff else 'following'))))


if __name__ == '__main__':
    rospy.init_node('rgbd_follower')
    Follower()
    rospy.spin()
