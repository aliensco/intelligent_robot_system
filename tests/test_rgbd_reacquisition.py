"""Recovery identity evidence must be consecutive and spatially relevant."""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.tracking import Tracker


class ReacquisitionTests(unittest.TestCase):
    def tracker(self):
        tracker = Tracker()
        for index in range(4):
            tracker.update([(1., 0., .3)], 1. + index * .1, [True])
        self.assertTrue(tracker.locked)
        tracker.update([], 1.4)
        return tracker

    def update(self, tracker, x, stamp, strong):
        return tracker.update([(x, 0., .3)], stamp, [strong])

    def test_weak_frames_cannot_complete_six_strong_confirmations(self):
        tracker = self.tracker()
        for index, x in enumerate((1.35, 1.36, 1.37)):
            self.assertIsNone(self.update(tracker, x, 3. + index * .1, False))
        for index, x in enumerate((1.50, 1.55, 1.60, 1.60, 1.60)):
            self.assertIsNone(self.update(tracker, x, 3.3 + index * .1, True))
            self.assertEqual(tracker.anchor, (1., 0., .3))
        self.assertEqual(self.update(tracker, 1.60, 3.8, True), (1.60, 0., .3))
        self.assertTrue(tracker.reacquired)

    def test_weak_frame_interrupts_strong_streak(self):
        tracker = self.tracker()
        for index in range(3):
            self.assertIsNone(self.update(tracker, 1.4, 3. + index * .1, True))
        self.assertIsNone(self.update(tracker, 1.4, 3.3, False))
        for index in range(5):
            self.assertIsNone(self.update(tracker, 1.5, 3.4 + index * .1, True))
        self.assertEqual(self.update(tracker, 1.5, 3.9, True), (1.5, 0., .3))

    def test_near_modality_switch_keeps_longer_confirmation_requirement(self):
        tracker = self.tracker()
        for index in range(3):
            self.assertIsNone(self.update(tracker, 1.1, 3. + index * .1, True))
        for index in range(2):
            self.assertIsNone(self.update(tracker, 1.1, 3.3 + index * .1, False))
        self.assertEqual(self.update(tracker, 1.1, 3.5, False), (1.1, 0., .3))

    def test_near_rgb_depth_strength_alternation_can_reacquire(self):
        tracker = self.tracker()
        for index in range(5):
            self.assertIsNone(self.update(tracker, 1.1, 3.+index*.1, index % 2 == 0))
            self.assertEqual(tracker.anchor, (1.,0.,.3))
        self.assertEqual(self.update(tracker, 1.1, 3.5, False),(1.1,0.,.3))
        self.assertTrue(tracker.reacquired)

    def test_near_mixed_streak_cannot_shortcut_far_strong_recovery(self):
        tracker = self.tracker()
        for index in range(5):
            self.assertIsNone(self.update(tracker, 1.1, 3.+index*.1, index % 2 == 0))
        for index in range(5):
            self.assertIsNone(self.update(tracker, 1.5, 3.5+index*.1, True))
        self.assertEqual(self.update(tracker, 1.5, 4., True),(1.5,0.,.3))

    def test_strong_distractor_outside_identity_radius_does_not_block(self):
        tracker = self.tracker()
        candidates = [(1.1, 0., .3), (4., 0., .3)]
        for index in range(3):
            self.assertIsNone(tracker.update(candidates, 3. + index * .1, [False, True]))
        self.assertEqual(tracker.update(candidates, 3.3, [False, True]), candidates[0])
        self.assertTrue(tracker.reacquired)

    def test_strong_distractor_outside_motion_gate_does_not_block(self):
        tracker = self.tracker()
        candidates = [(1.1, 0., .3), (2.45, 0., .3)]
        for index in range(3):
            self.assertIsNone(tracker.update(candidates, 1.5 + index * .1, [False, True]))
        self.assertEqual(tracker.update(candidates, 1.8, [False, True]), candidates[0])

    def test_weak_competitor_in_widened_region_preserves_ambiguity(self):
        tracker = self.tracker()
        # The strong near candidate must not hide the weaker but reachable one
        # beyond the ordinary .45 m radius when strong recovery widens the gate.
        candidates = [(1.1, 0., .3), (1.6, 0., .3)]
        for index in range(8):
            self.assertIsNone(tracker.update(candidates, 3. + index * .1, [True, False]))
            self.assertEqual(tracker.reason, 'ambiguous_target')
        self.assertEqual(tracker.anchor, (1., 0., .3))

    def test_unreachable_weak_candidate_does_not_block_strong_recovery(self):
        tracker = self.tracker()
        candidates = [(1.6, 0., .3), (4., 0., .3)]
        for index in range(5):
            self.assertIsNone(tracker.update(candidates, 3. + index * .1, [True, False]))
        self.assertEqual(tracker.update(candidates, 3.5, [True, False]), candidates[0])

    def test_miss_or_time_gap_requires_new_complete_strong_streak(self):
        for missing in (True, False):
            tracker = self.tracker()
            for index in range(3):
                self.assertIsNone(self.update(tracker, 1.6, 3. + index * .1, True))
            if missing:
                tracker.update([], 3.3)
            restart = 3.4 if missing else 4.
            for index in range(5):
                self.assertIsNone(self.update(tracker, 1.6, restart + index * .1, True))
            self.assertEqual(self.update(tracker, 1.6, restart + .5, True), (1.6, 0., .3))


if __name__ == '__main__':
    unittest.main()
