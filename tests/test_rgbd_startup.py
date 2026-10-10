"""Initial live standoff goals must retain the ordinary follower interlocks."""
import sys
import unittest

import test_rgbd_nodes as node_fixtures
from rgbd_following_core.tracking import Trail


@unittest.skipIf(sys.version_info[0] < 3, 'ROS doubles use Python 3')
class StartupTests(unittest.TestCase):
    def setUp(self):
        self.fixture = node_fixtures.NodeTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def follower(self, gap=1.2):
        n = self.fixture.follower()
        n.target = (1.5 + gap, 2.)
        n.trail = Trail()
        n.trail.add(n.target)
        return n

    def test_initial_goal_advances_only_excess_gap_without_inventing_trail(self):
        n = self.follower()
        original = list(n.trail.points)
        n.step()
        goal = n.client.send_goal.call_args[0][0].target_pose
        self.assertAlmostEqual(goal.pose.position.x, 1.7)
        self.assertAlmostEqual(goal.pose.position.y, 2.)
        self.assertEqual(n.trail.points, original)
        n.allowed.publish.assert_called_with(True)
        n.status.publish.assert_called_with('following_live_standoff')

    def test_target_below_follow_gap_never_generates_reverse_goal(self):
        n = self.follower(.9)
        n.step()
        n.client.send_goal.assert_not_called()
        n.allowed.publish.assert_called_with(False)
        n.status.publish.assert_called_with('waiting_for_target_trail')

    def test_target_below_minimum_gap_remains_blocked(self):
        n = self.follower(.3)
        n.step()
        n.client.send_goal.assert_not_called()
        n.status.publish.assert_called_with('target_too_close')

    def test_empty_trail_cannot_start(self):
        n = self.follower()
        n.trail = Trail()
        n.step()
        n.client.send_goal.assert_not_called()
        n.status.publish.assert_called_with('waiting_for_target_trail')

    def test_startup_keeps_target_odom_map_mission_and_estop_interlocks(self):
        for kind in ('target', 'odom', 'map', 'mission', 'estop'):
            n = self.follower()
            if kind == 'target':
                n.valid = False
            elif kind == 'odom':
                n.odometry.received -= 2.
            elif kind == 'map':
                n.grid_received -= 2.
            elif kind == 'mission':
                n.mission = 'idle'
            else:
                n.estop = True
            n.step()
            n.client.send_goal.assert_not_called()
            n.allowed.publish.assert_called_with(False)

    def test_startup_cannot_bypass_unknown_goal_or_missing_navigation(self):
        n = self.follower()
        n.grid.data[40 * 100 + 34] = -1
        n.step()
        n.client.send_goal.assert_not_called()
        n.status.publish.assert_called_with('follow_goal_blocked')
        n = self.follower()
        n.client.wait_for_server.return_value = False
        n.step()
        n.client.send_goal.assert_not_called()
        n.status.publish.assert_called_with('navigation_unavailable')

    def test_sufficient_real_trail_takes_priority_over_live_standoff(self):
        n = self.follower()
        n.target = (2.7, 2.5)
        n.trail.points = [(2., 2.), (2.5, 2.), (2.5, 2.5), n.target]
        n.step()
        goal = n.client.send_goal.call_args[0][0].target_pose
        self.assertAlmostEqual(goal.pose.position.x, 2.2)
        self.assertAlmostEqual(goal.pose.position.y, 2.)
        n.status.publish.assert_called_with('following')

    def test_initial_done_still_requires_finish_zone_stop_and_hold(self):
        n = self.follower(.95)
        n.mission = 'done'
        n.c['finish_polygon'] = []
        n.step()
        self.assertFalse(n.finished)
        n.status.publish.assert_called_with('finish_region_unconfigured')
        n.c['finish_polygon'] = [[0,0], [4,0], [4,4], [0,4]]
        n.odometry.speed = .1
        n.step()
        self.assertFalse(n.finished)
        self.assertIsNone(n.completion.since)
        n.odometry.speed = 0.
        n.step()
        self.assertFalse(n.finished)
        self.fixture.wall += 2.1
        n.target_received = n.valid_received = n.mission_received = n.grid_received = self.fixture.wall
        n.odometry.received = self.fixture.wall
        n.step()
        self.assertTrue(n.finished)
        n.client.send_goal.assert_not_called()


if __name__ == '__main__':
    unittest.main()
