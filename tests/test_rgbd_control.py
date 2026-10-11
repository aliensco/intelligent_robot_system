"""Geometry, identity association, completion and watchdog primitives (Py2/3)."""
import math
import os
import sys
import unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.tracking import Tracker, Trail, Completion, circle_inside_polygon
from rgbd_following_core.motion import Grid, swept_clear, need_goal, fresh


class AssociationTests(unittest.TestCase):
    def test_optional_empty_grace_preserves_association_but_never_returns_old_pose(self):
        tracker = Tracker(confirmations=2, empty_grace=.12)
        tracker.update([(1., 0., .2)], 1.)
        tracker.update([(1., 0., .2)], 1.1)
        self.assertIsNone(tracker.update([], 1.13))
        self.assertEqual(tracker.reason, 'observation_gap')
        self.assertEqual(tracker.stamp, 1.1)
        self.assertEqual(tracker.update([(1.01, 0., .2)], 1.16), (1.01, 0., .2))
        self.assertIsNone(tracker.update([], 1.4))
        self.assertEqual(tracker.count, 0)
        self.assertIsNone(tracker.update([(1.01, 0., .2)], 1.43))

    def test_empty_grace_does_not_relax_two_target_ambiguity(self):
        tracker = Tracker(confirmations=2, empty_grace=.12)
        tracker.update([(1., 0., .2)], 1.)
        tracker.update([(1., 0., .2)], 1.1)
        self.assertIsNone(tracker.update([(1.02, 0., .2), (.98, 0., .2)], 1.13))
        self.assertEqual(tracker.reason, 'ambiguous_target')

    def test_consecutive_confirmation_and_loss(self):
        tracker = Tracker(confirmations=3)
        self.assertIsNone(tracker.update([(1., 0., .3)], 1.))
        self.assertIsNone(tracker.update([(1.01, 0., .3)], 1.1))
        self.assertIsNotNone(tracker.update([(1.02, 0., .3)], 1.2))
        self.assertIsNone(tracker.update([], 1.3))
        self.assertIsNone(tracker.update([(1.03, 0., .3)], 1.4))

    def test_ambiguous_initialization_never_chooses_arbitrary_object(self):
        tracker = Tracker()
        self.assertIsNone(tracker.update([(1., 0., .2), (2., 0., .2)], 1.))
        self.assertEqual(tracker.reason, 'ambiguous_target')

    def test_crossing_similar_candidates_blocks(self):
        tracker = Tracker(confirmations=2)
        tracker.update([(1., 0., .2)], 1.)
        tracker.update([(1., 0., .2)], 1.1)
        self.assertIsNone(tracker.update([(1.02, 0., .2), (.98, 0., .2)], 1.2))
        self.assertEqual(tracker.reason, 'ambiguous_target')

    def test_far_distractor_does_not_steal_identity_after_loss(self):
        tracker = Tracker(confirmations=2)
        tracker.update([(1., 0., .2)], 1.)
        tracker.update([(1., 0., .2)], 1.1)
        self.assertIsNone(tracker.update([(3., 0., .2)], 30.))
        self.assertEqual(tracker.anchor, (1., 0., .2))

    def test_duplicate_and_large_frame_gap_reset_confirmation(self):
        tracker = Tracker(confirmations=2)
        tracker.update([(1., 0., .2)], 1.)
        self.assertIsNone(tracker.update([(1., 0., .2)], 1.))
        self.assertIsNone(tracker.update([(1., 0., .2)], 1.1))
        self.assertIsNone(tracker.update([(1., 0., .2)], 2.))

    def test_nonfinite_target_not_accepted(self):
        self.assertIsNone(Tracker().update([(float('nan'), 0., .2)], 1.))


class TrajectoryTests(unittest.TestCase):
    def test_two_turns_preserve_path_length(self):
        trail = Trail(jump=2.)
        for p in [(0, 0), (1, 0), (1, 1), (2, 1)]:
            self.assertTrue(trail.add(p))
        x, y, angle = trail.behind(1.5)
        self.assertEqual((x, y), (1., .5))
        self.assertAlmostEqual(angle, math.pi / 2)

    def test_waits_for_leader_and_rejects_jump(self):
        trail = Trail()
        trail.add((0, 0))
        self.assertIsNone(trail.behind(1.))
        self.assertFalse(trail.add((3., 0)))

    def test_final_goal_small_shift_is_delivered(self):
        self.assertTrue(need_goal((1, 0), (1.08, 0), 3, True, 1.))
        self.assertFalse(need_goal((1, 0), (1.08, 0), 1, False, 1.))

    def test_whole_body_must_be_inside_end_zone(self):
        poly = [[0, 0], [2, 0], [2, 2], [0, 2]]
        self.assertTrue(circle_inside_polygon((1, 1), .26, poly))
        self.assertFalse(circle_inside_polygon((.1, 1), .26, poly))
        self.assertFalse(circle_inside_polygon((1, 1), .26, []))

    def test_done_waits_for_stationary_target_and_zone(self):
        end = Completion(hold=2.)
        poly = [[0, 0], [4, 0], [4, 4], [0, 4]]
        args = ((1., 1.), (1., 1.), .15, poly, .26)
        self.assertFalse(end.ready('done', (2., 1.), 1., *(args+(True,))))
        self.assertFalse(end.ready('done', (2.2, 1.), 2.9, *(args+(True,))))
        self.assertFalse(end.ready('done', (2.2, 1.), 3., *(args+(True,))))
        self.assertTrue(end.ready('done', (2.2, 1.), 5., *(args+(True,))))
        end.reset()
        self.assertFalse(end.ready('done', (2.2, 1.), 6., *args))


class CollisionTests(unittest.TestCase):
    def grid(self):
        return Grid([0] * 6400, 80, 80, .05)

    def test_free_swept_path_and_reverse_rejection(self):
        grid = self.grid()
        self.assertTrue(swept_clear(grid, (2, 2, 0), .12, .2, .26))
        self.assertFalse(swept_clear(grid, (2, 2, 0), -.1, 0, .26))

    def test_obstacle_on_stopping_path_blocks_even_if_current_pose_clear(self):
        grid = self.grid()
        grid.data[40 * 80 + 47] = 100
        self.assertTrue(grid.clear(2., 2., .26))
        self.assertFalse(swept_clear(grid, (2, 2, 0), .12, 0, .26))

    def test_unknown_side_cell_blocks_rotation(self):
        grid = self.grid()
        grid.data[44 * 80 + 40] = -1
        self.assertFalse(swept_clear(grid, (2, 2, 0), 0, .4, .26))

    def test_boundaries_and_rotated_origin(self):
        grid = self.grid()
        self.assertFalse(grid.clear(.1, .1, .26))
        rotated = Grid(grid.data, 80, 80, .05, (4, 0, math.pi / 2))
        self.assertTrue(rotated.clear(2, 2, .26))

    def test_stale_future_and_nan_clock_inputs(self):
        for stamp in (None, 8., 11., float('nan')):
            self.assertFalse(fresh(stamp, 10., .5))


if __name__ == '__main__':
    unittest.main()
