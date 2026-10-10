#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Wall-clock watchdog + swept costmap footprint check, sole velocity output."""
import threading
import time
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, CameraInfo
from std_msgs.msg import Bool, String
from rgbd_following_core.depth import finite
from rgbd_following_core.motion import fresh, swept_clear, stopping_clear
from rgbd_following_core.visibility import visible_path
from rgbd_following_core.settings import validate
from rgbd_following_core.ros_helpers import grid_message, transform_pose, same_frame, OdometryState


class Gate(object):
    def __init__(self):
        self.execute = rospy.get_param('~execute', False)
        self.c = validate(rospy.get_param('~settings'), self.execute)
        self.frame, self.base = rospy.get_param('~odom_frame', 'odom'), rospy.get_param('~base_frame', 'base_footprint')
        self.clock = getattr(time, 'monotonic', time.time)
        self.lock = threading.RLock()
        self.command = Twist()
        self.command_received = self.allow_received = self.target_received = None
        self.cloud_stamp = self.cloud_received = self.grid_stamp = self.grid_received = None
        self.grid = None
        self.allowed = self.target_valid = self.estop = False
        self.reason = 'waiting'
        self.odometry = OdometryState()
        self.observation = self.observation_stamp = self.observation_received = None
        self.camera = self.camera_stamp = self.camera_received = self.camera_frame = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        # Shadow mode has no publisher on the base's navigation input.
        self.pub = rospy.Publisher('cmd_vel_safe' if self.execute else 'cmd_vel_preview', Twist, queue_size=1)
        self.status = rospy.Publisher('gate_status', String, queue_size=1)
        self.subs = [rospy.Subscriber('cmd_vel_raw', Twist, self.on_command, queue_size=1),
                     rospy.Subscriber('motion_allowed', Bool, self.on_allowed, queue_size=1),
                     rospy.Subscriber('target_valid', Bool, self.on_target, queue_size=1),
                     rospy.Subscriber('estop', Bool, self.on_estop, queue_size=1),
                     rospy.Subscriber('obstacle_cloud', PointCloud2, self.on_cloud, queue_size=1),
                     rospy.Subscriber('move_base/local_costmap/costmap', OccupancyGrid, self.on_grid, queue_size=1)]
        self.subs.extend([rospy.Subscriber('odom', Odometry, self.on_odom, queue_size=1),
                          rospy.Subscriber('target_pose', PoseStamped, self.on_pose, queue_size=1),
                          rospy.Subscriber('camera_info', CameraInfo, self.on_camera, queue_size=1)])
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()

    def on_command(self, msg):
        with self.lock:
            self.command, self.command_received = msg, self.clock()

    def on_odom(self, msg):
        with self.lock:
            self.odometry.update(msg, self.frame, self.base, rospy.Time.now().to_sec(), self.clock(), self.c['odom_timeout'])

    def on_pose(self, msg):
        with self.lock:
            stamp, p = msg.header.stamp.to_sec(), msg.pose.position
            if (same_frame(msg.header.frame_id, self.frame) and all(finite(v) for v in (p.x,p.y,p.z)) and
                    fresh(stamp, rospy.Time.now().to_sec(), self.c['target_timeout']) and
                    (self.observation_stamp is None or stamp > self.observation_stamp)):
                self.observation, self.observation_stamp, self.observation_received = (p.x,p.y,p.z), stamp, self.clock()

    def on_camera(self, msg):
        with self.lock:
            stamp, p = msg.header.stamp.to_sec(), msg.P
            if (len(p) == 12 and all(finite(v) for v in p) and min(p[0],p[5],msg.width,msg.height) > 0 and
                    0 <= p[2] < msg.width and 0 <= p[6] < msg.height and abs(p[3])+abs(p[7]) < 1e-6 and
                    msg.header.frame_id and fresh(stamp, rospy.Time.now().to_sec(), self.c['visibility']['camera_timeout']) and
                    (self.camera_stamp is None or stamp > self.camera_stamp)):
                self.camera = (p[0],p[5],p[2],p[6],msg.width,msg.height)
                self.camera_stamp, self.camera_received, self.camera_frame = stamp, self.clock(), msg.header.frame_id
            else:
                self.camera_received = None

    def on_allowed(self, msg):
        with self.lock:
            self.allowed, self.allow_received = msg.data, self.clock()
            if not msg.data:
                self.pub.publish(Twist())

    def on_target(self, msg):
        with self.lock:
            self.target_valid, self.target_received = msg.data, self.clock()
            if not msg.data:
                self.pub.publish(Twist())

    def on_estop(self, msg):
        with self.lock:
            self.estop = msg.data
            if msg.data:
                self.pub.publish(Twist())

    def on_cloud(self, msg):
        with self.lock:
            stamp = msg.header.stamp.to_sec()
            if (msg.width * msg.height >= self.c['perception']['min_cloud_points'] and
                    fresh(stamp, rospy.Time.now().to_sec(), self.c['target_timeout']) and
                    (self.cloud_stamp is None or stamp > self.cloud_stamp)):
                self.cloud_stamp, self.cloud_received = stamp, self.clock()

    def on_grid(self, msg):
        with self.lock:
            try:
                self.grid = grid_message(msg, self.frame)
                self.grid_stamp, self.grid_received = msg.header.stamp.to_sec(), self.clock()
            except ValueError:
                self.grid = None

    def output(self):
        c, wall, now = self.c, self.clock(), rospy.Time.now().to_sec()
        zero = Twist()
        self.reason = 'permission_or_command_stale'
        if (self.estop or not self.allowed or not self.target_valid or
                not fresh(self.command_received, wall, .3) or not fresh(self.allow_received, wall, .3) or
                not fresh(self.target_received, wall, c['target_timeout'])):
            return zero
        self.reason = 'depth_or_costmap_stale'
        if (self.grid is None or not fresh(self.cloud_received, wall, c['target_timeout']) or
                not fresh(self.cloud_stamp, now, c['target_timeout']) or
                not fresh(self.grid_received, wall, c['grid_timeout']) or
                not fresh(self.grid_stamp, now, c['grid_timeout'])):
            return zero
        v, w = self.command.linear.x, self.command.angular.z
        self.reason = 'invalid_navigation_velocity'
        all_values = (v, w, self.command.linear.y, self.command.linear.z, self.command.angular.x, self.command.angular.y)
        if not all(finite(x) for x in all_values) or v < 0 or any(abs(x) > 1e-6 for x in all_values[2:]):
            return zero
        # Reject oversized commands instead of independently clipping v/w and
        # changing the collision-checked curvature produced by DWA.
        if v > c['max_linear'] + 1e-6 or abs(w) > c['max_angular'] + 1e-6:
            return zero
        self.reason = 'odometry_missing_or_stale'
        if not self.odometry.valid(now, wall, c['odom_timeout']):
            return zero
        self.reason = 'visibility_input_stale'
        if (self.observation is None or self.camera is None or
                not fresh(self.observation_stamp, now, c['target_timeout']) or
                not fresh(self.observation_received, wall, c['target_timeout']) or
                not fresh(self.camera_stamp, now, c['visibility']['camera_timeout']) or
                not fresh(self.camera_received, wall, c['visibility']['camera_timeout'])):
            return zero
        self.reason = 'base_tf_invalid'
        try:
            tf = self.buffer.lookup_transform(self.frame, self.base, rospy.Time(0), rospy.Duration(.02))
            if not fresh(tf.header.stamp.to_sec(), now, c['tf_timeout']):
                return zero
            pose = transform_pose(tf)
            optical = self.buffer.lookup_transform(self.camera_frame, self.base, rospy.Time(0), rospy.Duration(.02))
            # Stamp zero is permitted only for a static camera extrinsic.
            if optical.header.stamp.to_sec() != 0 and not fresh(optical.header.stamp.to_sec(), now, c['tf_timeout']):
                return zero
            t, q = optical.transform.translation, optical.transform.rotation
            extrinsic = ((t.x,t.y,t.z), (q.x,q.y,q.z,q.w))
        except (ValueError, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return zero
        self.reason = 'measured_visibility_stopping_path_blocked'
        measured_horizon = c['reaction_time'] + max(self.odometry.speed/c['braking_acceleration'],
                                                   self.odometry.angular/c['angular_braking_acceleration'])
        measured_visible = visible_path(pose, self.observation, self.odometry.forward, self.odometry.yaw_rate,
                                        self.camera, extrinsic, measured_horizon, c['visibility']['margin'])
        # At rest near an image edge, a strict inner-margin test would reject
        # even a turn back toward the target. Permit only non-worsening measured
        # motion here; the command below must make visible recovery progress.
        recovery_ready = abs(self.odometry.forward) <= c['stopped_linear']
        if not measured_visible and recovery_ready:
            measured_visible = visible_path(pose, self.observation, self.odometry.forward, self.odometry.yaw_rate,
                                            self.camera, extrinsic, measured_horizon, c['visibility']['margin'],
                                            recovery=True, require_progress=False)
        if (abs(self.odometry.lateral) > c['stopped_linear'] or
                not measured_visible):
            return zero
        # Uniform scaling preserves curvature; every candidate is rechecked.
        # A future camera pose is evaluated with the current observed target.
        # This is a local visibility constraint, not an occlusion predictor.
        for scale in (1., .6, .3):
            vv, ww = v*scale, w*scale
            self.reason = 'visibility_path_blocked'
            horizon = max(c['visibility']['horizon'], c['reaction_time'] +
                          max(abs(vv)/c['braking_acceleration'], abs(ww)/c['angular_braking_acceleration']))
            visible = visible_path(pose, self.observation, vv, ww, self.camera, extrinsic,
                                   horizon, c['visibility']['margin'])
            recentering = False
            if not visible and recovery_ready and abs(vv) < 1e-6:
                recentering = visible_path(pose, self.observation, vv, ww, self.camera, extrinsic,
                                          horizon, c['visibility']['margin'], recovery=True)
            if not visible and not recentering:
                continue
            self.reason = 'measured_stopping_envelope_blocked'
            if not stopping_clear(self.grid, pose, self.odometry.speed, vv, c['footprint_radius'],
                                  c['braking_acceleration'], c['reaction_time']):
                continue
            self.reason = 'swept_footprint_blocked'
            if not swept_clear(self.grid, pose, vv, ww, c['footprint_radius'], c['braking_acceleration'], c['reaction_time']):
                continue
            self.reason = ('executing' if self.execute else 'shadow_velocity_only') if scale == 1 else 'visibility_or_clearance_slowdown'
            if recentering:
                self.reason = 'visibility_recenter' if self.execute else 'shadow_visibility_recenter'
            out = Twist()
            out.linear.x, out.angular.z = vv, ww
            return out
        return zero

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    self.pub.publish(self.output())
                    self.status.publish(self.reason)
                except Exception as exc:
                    self.pub.publish(Twist())
                    rospy.logerr_throttle(2, 'Gate error: ' + str(exc))
            time.sleep(.05)


if __name__ == '__main__':
    rospy.init_node('rgbd_velocity_gate')
    Gate()
    rospy.spin()
