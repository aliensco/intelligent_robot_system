"""Active visual recovery must remain a checked, fresh in-place movement."""
import sys
import unittest
import test_rgbd_nodes as node_fixtures


@unittest.skipIf(sys.version_info[0] < 3, 'ROS doubles use Python 3')
class AutoRecenterTests(unittest.TestCase):
    def setUp(self):
        self.fixture=node_fixtures.NodeTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def gate(self, side=1.):
        n=self.fixture.gate()
        n.observation=(2.5,2.+side*.5,.5)
        n.command.linear.x,n.command.angular.z=.04,0.
        return n

    def test_stationary_translating_request_recenters_either_visible_edge(self):
        for side in (-1.,1.):
            n=self.gate(side)
            out=n.output()
            self.assertEqual(out.linear.x,0.)
            self.assertGreater(out.angular.z*side,0.)
            self.assertLessEqual(abs(out.angular.z),n.c['max_angular'])
            self.assertEqual(n.reason,'shadow_visibility_recenter')

    def test_no_blind_search_after_loss_or_stale_inputs(self):
        for kind in ('target','observation','camera','command','map','depth','odom','permission'):
            n=self.gate()
            if kind=='target': n.target_valid=False
            elif kind=='observation': n.observation_received-=1.
            elif kind=='camera': n.camera_received-=2.
            elif kind=='command': n.command_received-=1.
            elif kind=='map': n.grid_received-=2.
            elif kind=='depth': n.cloud_received-=1.
            elif kind=='odom': n.odometry.received-=1.
            else: n.allowed=False
            out=n.output()
            self.assertEqual((out.linear.x,out.angular.z),(0.,0.),kind)

    def test_offscreen_target_and_unknown_or_lethal_footprint_do_not_recenter(self):
        for kind in ('offscreen','unknown','lethal'):
            n=self.gate()
            if kind=='offscreen': n.observation=(2.5,2.7,.5)
            else: n.grid.data[40*100+30]=-1 if kind=='unknown' else 100
            out=n.output()
            self.assertEqual((out.linear.x,out.angular.z),(0.,0.),kind)

    def test_recovery_starts_only_at_rest_but_can_continue_its_checked_inward_turn(self):
        n=self.gate()
        n.odometry.angular=n.odometry.yaw_rate=.15
        out=n.output()
        self.assertEqual((out.linear.x,out.angular.z),(0.,0.))
        n.odometry.angular=n.odometry.yaw_rate=0.
        self.assertGreater(n.output().angular.z,0.)
        n.odometry.angular=n.odometry.yaw_rate=.15
        self.assertGreater(n.output().angular.z,0.)
        n.odometry.yaw_rate=-.15
        out=n.output()
        self.assertEqual((out.linear.x,out.angular.z),(0.,0.))

    def test_no_recovery_during_forward_inertia(self):
        n=self.gate()
        n.odometry.speed=n.odometry.forward=.04
        out=n.output()
        self.assertEqual((out.linear.x,out.angular.z),(0.,0.))

    def test_centered_clear_navigation_remains_unchanged(self):
        n=self.gate()
        n.observation=(2.5,2.,.5)
        out=n.output()
        self.assertAlmostEqual(out.linear.x,.04)
        self.assertEqual(out.angular.z,0.)
        self.assertFalse(n.auto_recenter_active)

    def test_inner_but_near_edge_recovers_before_ineffective_scaled_translation(self):
        n=self.gate()
        n.observation=(2.5,2.4,.5)  # u=60, still inside the inner margin.
        n.command.angular.z=-.4
        out=n.output()
        self.assertEqual(out.linear.x,0.)
        self.assertGreater(out.angular.z,0.)

    def test_zero_navigation_goal_stop_never_starts_recovery(self):
        n=self.gate()
        n.command.linear.x=n.command.angular.z=0.
        out=n.output()
        self.assertEqual((out.linear.x,out.angular.z),(0.,0.))
        self.assertFalse(n.auto_recenter_active)


if __name__=='__main__':
    unittest.main()
