"""Collision regressions using costmap_2d's published inflation encoding."""
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.motion import Grid, stopping_clear, swept_clear


def published_cost(distance, inscribed=.31, inflation=.55):
    # Kinetic InflationLayer::computeCost and Costmap2DPublisher's translation.
    # Defaults match the project's .26 m robot radius + .05 m padding.
    if distance == 0:
        return 100
    if distance > inflation:
        return 0
    if distance <= inscribed:
        return 99
    raw = int(252 * math.exp(-4. * (distance - inscribed)))
    return 1 + (97 * (raw - 1)) // 251


def corridor(walls=(30, 50)):
    resolution, width, height = .05, 100, 100
    data = []
    for row in range(height):
        distance = min(abs(row - wall) for wall in walls) * resolution
        data.extend([published_cost(distance)] * width)
    return Grid(data, width, height, resolution)


class InflatedCollisionTests(unittest.TestCase):
    def test_one_meter_corridor_accepts_goal_and_low_speed_sweep(self):
        grid = corridor()
        # Wall centers y=1.525/2.525, robot centered y=2.025. Its footprint
        # overlaps 99-valued inflation but remains well away from real walls.
        pose = (2.025, 2.025, 0.)
        self.assertEqual(grid.data[35 * grid.width + 40], 99)
        self.assertTrue(grid.clear(pose[0], pose[1], .26))
        self.assertTrue(stopping_clear(grid, pose, .12, .12, .26, .3, .3))
        self.assertTrue(swept_clear(grid, pose, .12, .1, .26))

    def test_narrow_corridor_still_blocks(self):
        grid = corridor((35, 45))
        pose = (2.025, 2.025, 0.)
        self.assertFalse(grid.clear(pose[0], pose[1], .26))
        self.assertFalse(swept_clear(grid, pose, 0., .2, .26))

    def test_center_inscribed_exclusion_is_retained(self):
        grid = corridor()
        # A deliberately smaller explicit radius must not override the map's
        # padded center exclusion: this point is .30 m from the wall.
        self.assertFalse(grid.clear(2.025, 1.825, .10))

    def test_unknown_under_side_of_footprint_still_blocks_rotation(self):
        grid = corridor()
        grid.data[44 * grid.width + 40] = -1
        self.assertFalse(swept_clear(grid, (2.025, 2.025, 0.), 0., .2, .26))

    def test_lethal_cell_under_footprint_still_blocks(self):
        grid = corridor()
        grid.data[44 * grid.width + 40] = 100
        self.assertFalse(grid.clear(2.025, 2.025, .26))

    def test_measured_inertia_still_blocks_zero_command(self):
        grid = corridor()
        pose = (2.025, 2.025, 0.)
        self.assertTrue(stopping_clear(grid, pose, 0., 0., .26, .15, .3))
        self.assertFalse(stopping_clear(grid, pose, .3, 0., .26, .15, .3))

    def test_rotated_origin_preserves_costmap_semantics(self):
        original = corridor()
        grid = Grid(original.data, 100, 100, .05, (5., 0., math.pi / 2))
        self.assertTrue(grid.clear(5. - 2.025, 2.025, .26))
        self.assertFalse(grid.clear(5. - 1.825, 2.025, .10))


if __name__ == '__main__':
    unittest.main()
