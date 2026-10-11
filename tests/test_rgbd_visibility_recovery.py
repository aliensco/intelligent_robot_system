"""Edge recentering regressions; ROS doubles do not simulate the real base."""
import sys
import unittest

import test_rgbd_nodes as node_fixtures
from rgbd_following_core.visibility import visible_path


class VisibilityRecoveryTests(unittest.TestCase):
    def path(self, target=(1., .5, .5), w=.2, horizon=1., progress=True):
        return visible_path((0., 0., 0.), target, 0., w,
                            (250., 250., 160., 120., 320, 240),
                            ((0., .5, 0.), (.5, -.5, .5, .5)), horizon, .12,
                            recovery=True, require_progress=progress)

    def test_inward_turn_from_either_edge(self):
        self.assertTrue(self.path())
        self.assertTrue(self.path(target=(1., -.5, .5), w=-.2))

    def test_wrong_way_stationary_and_offscreen_are_not_recovery(self):
        self.assertFalse(self.path(w=-.2))
        self.assertFalse(self.path(w=0.))
        self.assertFalse(self.path(target=(1., .7, .5)))
        self.assertTrue(self.path(w=0., progress=False))

    def test_turn_cannot_overshoot_into_opposite_margin(self):
        self.assertFalse(self.path(w=1., horizon=1.2))

    def test_recovery_cannot_create_vertical_margin_violation(self):
        # This target starts inside the upper margin; the horizontal inward
        # turn crosses that margin through the perspective depth change.
        self.assertFalse(self.path(target=(1., .5, .863), w=1., horizon=.95))


@unittest.skipIf(sys.version_info[0] < 3, 'ROS doubles use Python 3')
class VisibilityRecoveryNodeTests(unittest.TestCase):
    def setUp(self):
        self.fixture = node_fixtures.NodeTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def gate(self):
        n = self.fixture.gate()
        n.observation = (2.5, 2.5, .5)  # u=35: in image, outside inner margin.
        n.command.linear.x, n.command.angular.z = 0., .2
        return n

    def test_fresh_edge_target_allows_collision_checked_inward_rotation(self):
        n = self.gate()
        out = n.output()
        self.assertEqual(out.linear.x, 0.)
        self.assertAlmostEqual(out.angular.z, .2)
        self.assertEqual(n.reason, 'shadow_visibility_recenter')

    def test_translating_curve_recovers_in_place_and_outward_spin_is_stopped(self):
        n = self.gate()
        n.command.linear.x, n.command.angular.z = .04, .2
        out = n.output()
        self.assertEqual(out.linear.x, 0.)
        self.assertGreater(out.angular.z, 0.)
        self.assertLessEqual(out.angular.z, n.c['max_angular'])
        n = self.gate()
        n.command.angular.z = -.2
        out = n.output()
        self.assertEqual((out.linear.x, out.angular.z), (0., 0.))

    def test_lost_stale_and_offscreen_targets_stay_stopped(self):
        for kind in ('lost', 'stale', 'offscreen'):
            n = self.gate()
            if kind == 'lost':
                n.target_valid = False
            elif kind == 'stale':
                n.observation_received -= 1.
            else:
                n.observation = (2.5, 2.7, .5)
            out = n.output()
            self.assertEqual((out.linear.x, out.angular.z), (0., 0.), kind)

    def test_measured_outward_motion_must_stop_before_recovery(self):
        n = self.gate()
        n.odometry.angular, n.odometry.yaw_rate = .2, -.2
        out = n.output()
        self.assertEqual((out.linear.x, out.angular.z), (0., 0.))
        self.assertEqual(n.reason, 'measured_visibility_stopping_path_blocked')

    def test_recovery_keeps_unknown_footprint_blocked(self):
        n = self.gate()
        n.grid.data[40 * 100 + 30] = -1
        out = n.output()
        self.assertEqual((out.linear.x, out.angular.z), (0., 0.))
        self.assertEqual(n.reason, 'measured_stopping_envelope_blocked')


if __name__ == '__main__':
    unittest.main()
