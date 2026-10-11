import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.motion import Grid, observed_prefix, clear_standoff


class PrefixTests(unittest.TestCase):
    def test_alternate_standoff_avoids_blocked_endpoint(self):
        g = Grid([0]*40000, 200, 200, .02, (-2., -2., 0.))
        g.data[100*200+130] = 100
        p = clear_standoff(g, (0., 0., 0.), (1.2, 0., 0.), .6, .12, .5)
        self.assertIsNotNone(p)
        self.assertTrue(g.clear(p[0], p[1], .12))
        gap = ((p[0]-1.2)**2 + p[1]**2)**.5
        self.assertGreaterEqual(gap, .5-1e-9)
        self.assertLessEqual(gap, .6+1e-9)

    def test_no_standoff_when_all_endpoints_blocked(self):
        g = Grid([100]*40000, 200, 200, .02, (-2., -2., 0.))
        self.assertIsNone(clear_standoff(g, (0., 0., 0.), (1.2, 0., 0.), .6, .12, .5))

    def grid(self):
        g = Grid([0]*10000, 100, 100, .02, (-1., -1., 0.))
        for iy in range(100):
            for ix in range(75, 100):
                g.data[iy*100+ix] = -1
        return g

    def test_stops_before_unknown_with_full_footprint(self):
        g = self.grid()
        p = observed_prefix(g, (0.,0.,0.), (.8,0.,0.), .26)
        self.assertIsNotNone(p)
        self.assertGreater(p[0], .1)
        self.assertLess(p[0], .25)
        self.assertTrue(g.clear(p[0],p[1],.26))

    def test_does_not_jump_over_obstacle_to_free_endpoint(self):
        g = Grid([0]*10000, 100, 100, .02, (-1.,-1.,0.))
        g.data[50*100+72] = 100
        p = observed_prefix(g, (0.,0.,0.), (.65,0.,0.), .12)
        self.assertLess(p[0], .33)

    def test_blocked_start_and_tiny_prefix_return_none(self):
        g = self.grid()
        g.data[50*100+50] = -1
        self.assertIsNone(observed_prefix(g,(0.,0.,0.),(.8,0.,0.),.26))
        self.assertIsNone(observed_prefix(self.grid(),(0.,0.,0.),(.03,0.,0.),.26))


if __name__ == '__main__':
    unittest.main()
