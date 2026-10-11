"""Follower/gate adapters with ROS doubles; real navigation is not simulated."""
import copy
import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))


@unittest.skipIf(sys.version_info[0] < 3, 'ROS doubles use Python 3')
class NodeTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        import threading
        import yaml
        # Load native extension modules before patch.dict(sys.modules), whose
        # cleanup would otherwise remove a freshly imported NumPy/OpenCV runtime.
        try:
            import numpy
            import cv2
        except ImportError:
            pass
        from types import ModuleType, SimpleNamespace as NS
        from unittest.mock import Mock, patch
        self.NS, self.Mock = NS, Mock
        self.now, self.wall = 100., 200.
        def mod(name, **values):
            result = ModuleType(name)
            result.__dict__.update(values)
            return result
        owner = self
        class Stamp(object):
            def __init__(self, value):
                self.value = value
            def to_sec(self):
                return self.value
        class Time(object):
            def __init__(self, value=0):
                self.value = value
            @staticmethod
            def now():
                return Stamp(owner.now)
        self.Stamp = Stamp
        self.twist = lambda: NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.))
        self.pose = lambda: NS(header=NS(stamp=Stamp(self.now), frame_id='odom'),
                              pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
        fake = {
            'rospy': mod('rospy', Time=Time, Duration=lambda v: v),
            'tf2_ros': mod('tf2_ros', LookupException=LookupError, ConnectivityException=ConnectionError, ExtrapolationException=ValueError),
            'actionlib': mod('actionlib'),
            'geometry_msgs': mod('geometry_msgs'),
            'geometry_msgs.msg': mod('geometry_msgs.msg', Twist=self.twist, PoseStamped=self.pose),
            'nav_msgs': mod('nav_msgs'),
            'nav_msgs.msg': mod('nav_msgs.msg', OccupancyGrid=object, Odometry=object),
            'sensor_msgs': mod('sensor_msgs'),
            'sensor_msgs.msg': mod('sensor_msgs.msg', PointCloud2=object, CameraInfo=object),
            'std_msgs': mod('std_msgs'),
            'std_msgs.msg': mod('std_msgs.msg', Bool=bool, String=str),
            'move_base_msgs': mod('move_base_msgs'),
            'move_base_msgs.msg': mod('move_base_msgs.msg', MoveBaseAction=object, MoveBaseGoal=lambda: NS(target_pose=self.pose()))
        }
        patcher = patch.dict(sys.modules, fake)
        patcher.start()
        self.addCleanup(patcher.stop)
        def load(name):
            path = os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/scripts', name + '.py')
            spec = importlib.util.spec_from_file_location('adapter_test_' + name, path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
        self.follow_module, self.gate_module = load('rgbd_follower'), load('rgbd_velocity_gate')
        with open(os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/config/follow.yaml')) as f:
            self.c = yaml.safe_load(f)
        self.c['finish_polygon'] = [[0, 0], [4, 0], [4, 4], [0, 4]]
        self.lock = threading.RLock()

    def base(self, cls):
        from rgbd_following_core.motion import Grid
        n = cls.__new__(cls)
        n.lock = self.lock
        n.c, n.frame, n.base = copy.deepcopy(self.c), 'odom', 'base_footprint'
        n.clock = lambda: self.wall
        n.grid = Grid([0] * 10000, 100, 100, .05)
        n.grid_stamp, n.grid_received = self.now, self.wall
        n.buffer = self.Mock()
        n.buffer.lookup_transform.return_value = self.NS(header=self.NS(stamp=self.Stamp(self.now)),
            transform=self.NS(translation=self.NS(x=1.5, y=2., z=0.), rotation=self.NS(x=0., y=0., z=0., w=1.)))
        n.estop = False
        from rgbd_following_core.ros_helpers import OdometryState
        n.odometry = OdometryState()
        n.odometry.stamp, n.odometry.received = self.now, self.wall
        return n

    def follower(self):
        from rgbd_following_core.tracking import Trail, Completion
        n = self.base(self.follow_module.Follower)
        n.target, n.target_stamp, n.target_received = (3., 2.), self.now, self.wall
        n.valid, n.valid_received, n.mission_received = True, self.wall, self.wall
        n.mission = 'carrying'
        n.trail = Trail(jump=2.)
        n.trail.add((2., 2.))
        n.trail.add((3., 2.))
        n.completion = Completion()
        n.goal, n.goal_time = None, 0.
        n.lost_since = n.failed = n.previous_pose = None
        n.finished = False
        n.trail_interrupted = n.rebuilding_trail = False
        n.client, n.allowed, n.status = self.Mock(), self.Mock(), self.Mock()
        n.client.wait_for_server.return_value = True
        n.client.get_state.return_value = 1
        return n

    def gate(self):
        n = self.base(self.gate_module.Gate)
        n.execute = False
        n.command = self.twist()
        n.command.linear.x = .1
        n.command_received = n.allow_received = n.target_received = self.wall
        n.cloud_stamp, n.cloud_received = self.now, self.wall
        n.allowed = n.target_valid = True
        n.pub = self.Mock()
        n.observation, n.observation_stamp, n.observation_received = (3.,2.,.5), self.now, self.wall
        n.camera = (250.,250.,160.,120.,320,240)
        n.camera_stamp, n.camera_received, n.camera_frame = self.now, self.wall, 'camera_optical'
        base_tf = n.buffer.lookup_transform.return_value
        optical_tf = self.NS(header=self.NS(stamp=self.Stamp(0.)), transform=self.NS(
            translation=self.NS(x=0.,y=.5,z=0.), rotation=self.NS(x=.5,y=-.5,z=.5,w=.5)))
        n.buffer.lookup_transform.side_effect = lambda parent, *args: optical_tf if parent == 'camera_optical' else base_tf
        return n

    def test_navigation_receives_measured_trailing_goal(self):
        n = self.follower()
        n.step()
        goal = n.client.send_goal.call_args[0][0].target_pose
        self.assertEqual(goal.header.frame_id, 'odom')
        self.assertEqual(goal.pose.position.x, 2.)
        n.allowed.publish.assert_called_with(True)

    def test_lost_target_cancels_goal_and_never_searches_by_motion(self):
        n = self.follower()
        n.step()
        n.valid = False
        n.step()
        n.allowed.publish.assert_called_with(False)
        n.client.cancel_goal.assert_called_once()
        n.status.publish.assert_called_with('target_lost_hold_position')

    def test_sixty_second_loss_latches_failure(self):
        n = self.follower()
        n.valid = False
        n.step()
        self.wall += 61
        n.mission_received = self.wall
        n.step()
        self.assertEqual(n.failed, 'target_lost_over_limit_reset_required')

    def test_final_small_goal_change_not_ignored(self):
        n = self.follower()
        n.mission = 'done'
        n.goal, n.goal_time = (2., 2., 0.), self.wall - 1
        n.trail.add((3.08, 2.))
        n.target = (3.08, 2.)
        n.step()
        self.assertAlmostEqual(n.client.send_goal.call_args[0][0].target_pose.pose.position.x, 2.08)

    def test_done_requires_hold_time_and_finish_zone_then_latches(self):
        n = self.follower()
        n.mission = 'done'
        n.buffer.lookup_transform.return_value.transform.translation.x = 2.
        n.step()
        self.assertFalse(n.finished)
        self.wall += 2.1
        n.target_received = n.valid_received = n.mission_received = n.grid_received = self.wall
        n.odometry.received = self.wall
        n.step()
        self.assertTrue(n.finished)
        n.allowed.publish.assert_called_with(False)
        n.valid = False
        n.step()
        n.status.publish.assert_called_with('finished')

    def test_missing_zone_never_finishes(self):
        n = self.follower()
        n.mission, n.c['finish_polygon'] = 'done', []
        n.buffer.lookup_transform.return_value.transform.translation.x = 2.
        n.step()
        n.status.publish.assert_called_with('finish_region_unconfigured')
        self.assertFalse(n.finished)

    def test_navigation_abort_latches(self):
        n = self.follower()
        n.goal = (2., 2., 0.)
        n.client.get_state.return_value = 4
        n.step()
        self.assertEqual(n.failed, 'navigation_failed_reset_required')
        n.allowed.publish.assert_called_with(False)

    def test_field_navigation_retry_requires_new_map_and_selects_another_goal(self):
        n = self.follower()
        n.c.update(navigation_retry_delay=.8, detour_max_extra_distance=.6, standoff_min_distance=.8)
        n.goal = (2., 2., 0.)
        n.on_navigation_done(4, None)
        self.assertIsNone(n.failed)
        n.step()
        n.status.publish.assert_called_with('navigation_retry_wait')
        n.client.send_goal.assert_not_called()
        self.wall += .9
        n.target_received = n.valid_received = n.mission_received = n.grid_received = self.wall
        n.odometry.received = self.wall
        n.step()
        n.status.publish.assert_called_with('navigation_retry_wait')
        n.grid_stamp += .001
        self.now += .001
        n.step()
        self.assertIsNone(n.failed)
        goal = n.client.send_goal.call_args[0][0].target_pose.pose.position
        self.assertGreaterEqual(((goal.x-2.)**2+(goal.y-2.)**2)**.5, .15)

    def test_field_retry_does_not_override_estop_or_lost_action_connection(self):
        n = self.follower()
        n.c['navigation_retry_delay'] = .8
        n.goal = (2.,2.,0.)
        n.on_navigation_done(4, None)
        n.estop = True
        n.step()
        n.status.publish.assert_called_with('mission_stopped')
        n.client.send_goal.assert_not_called()
        n.on_navigation_done(9,None)
        self.assertEqual(n.failed, 'navigation_failed_reset_required')

    def test_field_clear_endpoint_still_uses_reachable_approach(self):
        from unittest.mock import patch
        n = self.follower()
        n.c.update(detour_max_extra_distance=.6, standoff_min_distance=.8)
        self.assertTrue(n.grid.clear(2., 2., n.c['footprint_radius']))
        with patch.object(self.follow_module, 'reachable_standoff',
                          return_value=((1.7, 2.1, 0.), True)) as planner:
            n.step()
        planner.assert_called_once()
        goal = n.client.send_goal.call_args[0][0].target_pose.pose.position
        self.assertAlmostEqual(goal.x, 1.7)
        self.assertAlmostEqual(goal.y, 2.1)

    def test_obsolete_goal_callback_cannot_abort_a_new_goal(self):
        n = self.follower()
        n.goal_sequence = 2
        n.on_navigation_done(4, None, sequence=1)
        self.assertIsNone(n.failed)

    def test_persistent_visibility_veto_replans_instead_of_repeating_same_turn(self):
        n = self.follower()
        n.c['navigation_retry_delay'] = .8
        n.goal = (2., 2., 0.)
        for unused in range(25):
            self.advance_follower(n)
            n.on_gate(self.NS(data='visibility_path_blocked'))
            n.step()
            if getattr(n, 'retry_pending', False):
                break
        self.assertTrue(n.retry_pending)
        self.assertIsNone(n.failed)
        self.assertEqual(n.failed_goal, (2., 2., 0.))
        n.client.cancel_goal.assert_called_once()
        n.allowed.publish.assert_called_with(False)
        n.status.publish.assert_called_with('navigation_retry_no_progress')

    def advance_follower(self, n, elapsed=.1, position=None, heading=None):
        self.wall += elapsed
        self.now += elapsed
        n.target_stamp = n.grid_stamp = n.odometry.stamp = self.now
        n.target_received = n.valid_received = n.mission_received = n.grid_received = self.wall
        n.odometry.received = self.wall
        transform = n.buffer.lookup_transform.return_value
        transform.header.stamp = self.Stamp(self.now)
        if position is not None:
            transform.transform.translation.x, transform.transform.translation.y = position
        if heading is not None:
            transform.transform.rotation.z = math.sin(heading/2.)
            transform.transform.rotation.w = math.cos(heading/2.)

    def test_intermittent_gate_slowdown_cannot_hide_a_stationary_goal(self):
        n = self.follower()
        n.c['navigation_retry_delay'] = .8
        n.goal = (2., 2., 0.)
        for index in range(50):
            self.advance_follower(n)
            # Regression for the field v=.01,w=-.09 friction stall: occasional
            # healthy status messages must not reset all slowdown evidence.
            reason = 'visibility_or_clearance_slowdown' if index % 2 == 0 else 'shadow_velocity_only'
            n.on_gate(self.NS(data=reason))
            n.step()
            if getattr(n, 'retry_pending', False):
                break
        self.assertTrue(n.retry_pending)
        self.assertEqual(n.excluded_goals[-1][0], (2., 2., 0.))
        n.client.cancel_goal.assert_called_once()
        self.assertIsNone(n.failed)

    def test_slow_translation_or_real_inplace_rotation_is_useful_progress(self):
        for motion in ('translation', 'rotation'):
            n = self.follower()
            n.c['navigation_retry_delay'] = .8
            n.goal = (2., 2., 0.)
            for index in range(50):
                self.advance_follower(n, position=(1.5+.003*index, 2.) if motion == 'translation' else None,
                                      heading=.01*index if motion == 'rotation' else None)
                n.on_gate(self.NS(data='visibility_or_clearance_slowdown'))
                n.step()
            n.client.cancel_goal.assert_not_called()
            self.assertFalse(getattr(n, 'retry_pending', False))

    def test_normal_gap_or_arrived_goal_wait_is_not_a_friction_stall(self):
        for waiting in ('gap', 'arrived'):
            n = self.follower()
            n.c['navigation_retry_delay'] = .8
            n.goal = (2., 2., 0.)
            if waiting == 'gap':
                n.target = (2.5, 2.)
            else:
                n.client.get_state.return_value = 3
            for unused in range(30):
                self.advance_follower(n)
                n.on_gate(self.NS(data='visibility_or_clearance_slowdown'))
                n.step()
            n.client.cancel_goal.assert_not_called()
            self.assertFalse(getattr(n, 'retry_pending', False))

    def test_no_progress_retry_requires_fresh_gate_evidence(self):
        n = self.follower()
        n.c['navigation_retry_delay'] = .8
        n.goal = (2., 2., 0.)
        n.progress_anchor = (1.5,2.,0.)
        n.gate_limited_seconds = 3.
        n.gate_last_limited = self.wall
        n.gate_received = self.wall-1.
        n.step()
        n.client.cancel_goal.assert_not_called()
        self.assertFalse(getattr(n, 'retry_pending', False))

    def test_transient_visibility_veto_does_not_cancel_goal(self):
        n = self.follower()
        n.c['navigation_retry_delay'] = .8
        n.goal = (2., 2., 0.)
        n.on_gate(self.NS(data='visibility_path_blocked'))
        self.wall += .3
        n.on_gate(self.NS(data='shadow_velocity_only'))
        self.wall += 1.
        n.on_gate(self.NS(data='visibility_path_blocked'))
        n.client.cancel_goal.assert_not_called()

    def test_reached_detour_is_excluded_and_old_exclusions_expire(self):
        n = self.follower()
        n.goal, n.goal_is_detour = (1.5, 2., 0.), True
        n.step()
        self.assertEqual(n.excluded_goals[0][0], (1.5, 2., 0.))
        n.excluded_goals[0] = (n.excluded_goals[0][0], self.wall-11., n.target)
        n.step()
        self.assertEqual(n.excluded_goals, [])

    def test_usable_alternate_goal_is_held_but_a_moving_target_releases_it(self):
        n = self.follower()
        n.c['detour_goal_hold'] = 3.
        n.goal, n.goal_time = (2., 1.7, 0.), self.wall-1.
        n.goal_is_alternate, n.goal_is_detour = True, False
        n.goal_target_anchor = n.target
        n.step()
        n.client.send_goal.assert_not_called()
        n.target = (3.2, 2.)
        n.step()
        n.client.send_goal.assert_called_once()

    def test_goal_hold_never_reuses_a_newly_blocked_endpoint(self):
        n = self.follower()
        n.c['detour_goal_hold'] = 3.
        n.goal, n.goal_time = (2., 1.7, 0.), self.wall-1.
        n.goal_is_alternate, n.goal_is_detour = True, False
        n.goal_target_anchor = n.target
        n.grid = self.Mock()
        n.grid.clear.side_effect = lambda x, y, radius: y > 1.9
        n.step()
        n.client.send_goal.assert_called_once()
        self.assertEqual(n.goal[:2], (2., 2.))

    def test_terminal_callback_latches_failure_even_after_goal_cancel(self):
        n = self.follower()
        n.goal = (2.,2.,0.)
        n.on_valid(self.NS(data=False))
        n.on_navigation_done(4, None)
        n.valid = True
        n.step()
        self.assertEqual(n.failed, 'navigation_failed_reset_required')
        n.client.send_goal.assert_not_called()

    def test_reacquisition_does_not_bridge_unobserved_target_path(self):
        n = self.follower()
        n.on_valid(self.NS(data=False))
        self.now += 2.
        msg = self.pose()
        msg.pose.position.x, msg.pose.position.y = 3.8,2.
        n.on_target(msg)
        self.assertIsNone(n.failed)
        self.assertEqual(n.trail.points, [(3.8,2.)])
        self.assertTrue(n.rebuilding_trail)

    def test_gate_rejects_stale_odom_and_scales_turn_away_from_target(self):
        n = self.gate()
        n.odometry.received -= 1.
        self.assertEqual(n.output().linear.x, 0.)
        self.assertEqual(n.reason, 'odometry_missing_or_stale')
        n = self.gate()
        n.observation = (2.5,2.18,.5)
        n.command.angular.z = -.4
        out = n.output()
        self.assertGreater(out.linear.x,0.)
        self.assertLess(out.linear.x,.1)
        self.assertAlmostEqual(out.angular.z/out.linear.x,-4.)

    def test_camera_duplicate_and_stale_pose_do_not_refresh_gate(self):
        n = self.gate()
        n.observation_stamp -= 2.
        self.assertEqual(n.output().linear.x,0.)
        self.assertEqual(n.reason,'visibility_input_stale')

    def test_measured_rotation_must_stop_before_visibility_boundary(self):
        n = self.gate()
        n.observation = (2.5,2.18,.5)
        n.odometry.angular, n.odometry.yaw_rate = .7,-.7
        n.command.angular.z = -.04
        self.assertEqual(n.output().linear.x,0.)
        self.assertEqual(n.reason,'measured_visibility_stopping_path_blocked')

    def test_odometry_rejects_wrong_frame_duplicate_and_nonfinite_velocity(self):
        from rgbd_following_core.ros_helpers import OdometryState
        state = OdometryState()
        msg = self.NS(header=self.NS(stamp=self.Stamp(self.now),frame_id='odom'),
                      child_frame_id='base_footprint',twist=self.NS(twist=self.twist()))
        msg.twist.twist.linear.x = .12
        self.assertTrue(state.update(msg,'odom','base_footprint',self.now,self.wall,.3))
        self.assertAlmostEqual(state.speed,.12)
        self.assertFalse(state.update(msg,'odom','base_footprint',self.now,self.wall,.3))
        self.assertFalse(state.valid(self.now,self.wall,.3))
        msg.header.stamp = self.Stamp(self.now+.1)
        msg.child_frame_id = 'other_robot'
        self.assertFalse(state.update(msg,'odom','base_footprint',self.now+.1,self.wall,.3))
        msg.child_frame_id = 'base_footprint'
        msg.twist.twist.linear.x = float('nan')
        self.assertFalse(state.update(msg,'odom','base_footprint',self.now+.1,self.wall,.3))

    def test_follower_does_not_finish_while_odom_reports_motion(self):
        n = self.follower()
        n.mission = 'done'
        n.buffer.lookup_transform.return_value.transform.translation.x = 2.
        n.odometry.speed = .1
        for unused in range(30):
            self.wall += .1
            n.target_received = n.valid_received = n.mission_received = n.grid_received = n.odometry.received = self.wall
            n.step()
        self.assertFalse(n.finished)

    def test_reacquired_stationary_target_can_finish_within_gap_tolerance(self):
        from rgbd_following_core.tracking import Trail
        n = self.follower()
        n.mission, n.rebuilding_trail = 'done', True
        n.target, n.trail = (2.45,2.), Trail()
        n.trail.add(n.target)
        n.step()
        self.assertFalse(n.finished)
        self.wall += 2.1
        n.target_received = n.valid_received = n.mission_received = n.grid_received = n.odometry.received = self.wall
        n.step()
        self.assertTrue(n.finished)

    def test_abort_is_not_erased_by_simultaneous_target_loss(self):
        n = self.follower()
        n.goal = (2., 2., 0.)
        n.client.get_state.return_value = 4
        n.on_valid(self.NS(data=False))
        n.step()
        self.assertEqual(n.failed, 'navigation_failed_reset_required')

    def test_shadow_constructor_has_no_hardware_publisher(self):
        from unittest.mock import patch
        module = self.gate_module
        names = []
        def publisher(name, *args, **kwargs):
            names.append(name)
            return self.Mock()
        def parameter(name, default=None):
            return self.c if name == '~settings' else default
        with patch.object(module.rospy, 'get_param', parameter, create=True), \
             patch.object(module.rospy, 'Publisher', publisher, create=True), \
             patch.object(module.rospy, 'Subscriber', self.Mock(), create=True), \
             patch.object(module.rospy, 'on_shutdown', self.Mock(), create=True), \
             patch.object(module.tf2_ros, 'Buffer', self.Mock(), create=True), \
             patch.object(module.tf2_ros, 'TransformListener', self.Mock(), create=True), \
             patch.object(module.threading, 'Thread', self.Mock()):
            module.Gate()
        self.assertIn('cmd_vel_preview', names)
        self.assertNotIn('cmd_vel_safe', names)

    def test_unvalidated_execute_is_rejected_before_publisher_creation(self):
        from unittest.mock import patch
        module = self.gate_module
        publisher = self.Mock()
        def parameter(name, default=None):
            return self.c if name == '~settings' else (True if name == '~execute' else default)
        with patch.object(module.rospy, 'get_param', parameter, create=True), \
             patch.object(module.rospy, 'Publisher', publisher, create=True):
            with self.assertRaises(ValueError):
                module.Gate()
        publisher.assert_not_called()

    def test_shadow_command_is_computed_and_unsafe_commands_are_zero(self):
        n = self.gate()
        self.assertAlmostEqual(n.output().linear.x, .1)
        self.assertEqual(n.reason, 'shadow_velocity_only')
        for v, w in ((-.1, 0), (.2, 0), (.1, .9), (float('nan'), 0)):
            n.command.linear.x, n.command.angular.z = v, w
            out = n.output()
            self.assertEqual((out.linear.x, out.angular.z), (0., 0.))

    def test_gate_all_watchdogs_and_estop(self):
        for attr in ('command_received', 'allow_received', 'target_received', 'cloud_received', 'grid_received'):
            n = self.gate()
            setattr(n, attr, self.wall - 2)
            self.assertEqual(n.output().linear.x, 0., attr)
        n = self.gate()
        n.on_estop(self.NS(data=True))
        self.assertEqual(n.output().linear.x, 0.)
        self.assertEqual(n.pub.publish.call_count, 1)

    def test_replayed_cloud_cannot_refresh_watchdog(self):
        n = self.gate()
        self.wall += .1
        n.on_cloud(self.NS(width=1000, height=1, header=self.NS(stamp=self.Stamp(self.now))))
        self.assertEqual(n.cloud_received, 200.)

    def test_gate_stops_for_unknown_swept_footprint(self):
        n = self.gate()
        n.grid.data[40 * 100 + 30] = -1
        self.assertEqual(n.output().linear.x, 0.)
        self.assertEqual(n.reason, 'measured_stopping_envelope_blocked')

    def test_perception_callback_ignores_duplicate_without_extending_freshness(self):
        import importlib.util
        from types import ModuleType
        from unittest.mock import patch
        try:
            import numpy as np
            import cv2
        except ImportError:
            self.skipTest('Install OpenCV and NumPy for perception adapter test')
        fake = {}
        bridge_module = ModuleType('cv_bridge')
        bridge_module.CvBridge, bridge_module.CvBridgeError = self.Mock, TypeError
        fake['cv_bridge'] = bridge_module
        fake['message_filters'] = ModuleType('message_filters')
        clouds = ModuleType('sensor_msgs.point_cloud2')
        clouds.create_cloud_xyz32 = lambda header, points: self.NS(header=header, points=points)
        fake['sensor_msgs.point_cloud2'] = clouds
        with patch.dict(sys.modules, fake), \
             patch.object(sys.modules['rospy'], 'logwarn_throttle', self.Mock(), create=True), \
             patch.object(sys.modules['sensor_msgs.msg'], 'Image', object, create=True), \
             patch.object(sys.modules['sensor_msgs.msg'], 'CameraInfo', object, create=True), \
             patch.object(sys.modules['std_msgs.msg'], 'String', lambda data: self.NS(data=data)):
            path = os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/scripts/rgbd_target.py')
            spec = importlib.util.spec_from_file_location('target_adapter_test', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            from rgbd_following_core.tracking import Tracker
            n = module.Target.__new__(module.Target)
            n.lock, n.config, n.frame = self.lock, self.c, 'odom'
            n.clock = lambda: self.wall
            n.last_pair = n.receipt = None
            n.tracker = Tracker(**self.c['tracker'])
            n.bridge, n.buffer = self.Mock(), self.Mock()
            n.pub, n.valid, n.cloud, n.clearing, n.status, n.preview = [self.Mock() for unused in range(6)]
            rgb = np.zeros((240, 320, 3), np.uint8)
            rgb[90:150, 130:190] = (0, 220, 0)
            depth = np.full((240, 320), 2.5, np.float32)
            depth[90:150, 130:190] = 1.2
            n.bridge.imgmsg_to_cv2.side_effect = lambda msg, encoding: rgb if encoding == 'bgr8' else depth
            n.buffer.lookup_transform.return_value = self.NS(transform=self.NS(
                translation=self.NS(x=1.5, y=2., z=.5), rotation=self.NS(x=-.5, y=.5, z=-.5, w=.5)))
            messages = [self.NS(width=320, height=240, encoding='32FC1',
                header=self.NS(frame_id='rgb_optical', stamp=self.Stamp(self.now))) for unused in range(3)]
            messages[2].P = [250., 0., 160., 0., 0., 250., 120., 0., 0., 0., 1., 0.]
            for unused in range(4):
                self.now += .1
                for message in messages:
                    message.header.stamp = self.Stamp(self.now)
                n.callback(*messages)
            n.pub.publish.assert_called_once()
            n.valid.publish.assert_called_with(True)
            evidence = json.loads(n.status.publish.call_args[0][0].data)
            self.assertEqual(evidence['selected_candidate'], 0)
            self.assertEqual(evidence['candidate_sources'], [['color']])
            self.assertGreaterEqual(evidence['detection_seconds'], 0.)
            self.assertGreaterEqual(evidence['observation_age_seconds'], 0.)
            target = n.pub.publish.call_args[0][0]
            self.assertEqual(target.header.frame_id, 'odom')
            self.assertAlmostEqual(target.pose.position.x, 2.7, places=5)
            self.assertEqual(n.cloud.publish.call_count, 4)
            self.assertEqual(n.clearing.publish.call_count, 4)
            self.assertIs(n.clearing.publish.call_args[0][0].header, messages[1].header)
            receipt, pair = n.receipt, n.last_pair[:]
            publish_count = n.valid.publish.call_count
            self.wall += .1
            n.callback(*messages)
            n.valid.publish.assert_called_with(True)
            self.assertEqual(n.valid.publish.call_count, publish_count)
            self.assertEqual(n.receipt, receipt)
            self.assertEqual(n.last_pair, pair)
            self.assertEqual(n.cloud.publish.call_count, 4)
            self.assertEqual(n.clearing.publish.call_count, 4)
            self.assertTrue(json.loads(n.status.publish.call_args[0][0].data)['target_valid'])
            # Freshness failure must not publish another clearing cloud either.
            self.now += 1.
            n.callback(*messages)
            n.valid.publish.assert_called_with(False)
            self.assertEqual(n.clearing.publish.call_count, 4)
            depth[:] = 4.3
            for message in messages:
                message.header.stamp = self.Stamp(self.now)
            n.callback(*messages)
            self.assertEqual(n.clearing.publish.call_count, 5)
            self.assertTrue(n.clearing.publish.call_args[0][0].points)
            self.assertEqual(n.cloud.publish.call_count, 5)
            self.assertFalse(n.cloud.publish.call_args[0][0].points)
            n.valid.publish.assert_called_with(False)
            self.assertEqual(json.loads(n.status.publish.call_args[0][0].data)['reason'], 'depth_coverage_insufficient')
            # Sparse real foreground must replace old marking points too,
            # even above the point-count threshold but below coverage ratio.
            depth[:40, :80] = 1.2
            self.now += .1
            for message in messages:
                message.header.stamp = self.Stamp(self.now)
            n.callback(*messages)
            self.assertEqual(n.cloud.publish.call_count, 6)
            self.assertTrue(n.cloud.publish.call_args[0][0].points)
            n.valid.publish.assert_called_with(False)
            self.assertEqual(json.loads(n.status.publish.call_args[0][0].data)['reason'], 'depth_coverage_insufficient')


if __name__ == '__main__':
    unittest.main()
