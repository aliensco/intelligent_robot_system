"""Processing backlog must not hold up fresh synchronized camera frames."""
import os
import sys
import threading
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)),
                              'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.latest_pair import LatestPair


class LatestPairTests(unittest.TestCase):
    def test_slow_detector_receives_latest_not_accumulated_history(self):
        pairs = LatestPair()
        entered, finish = threading.Event(), threading.Event()
        processed = []
        pairs.put('first', (1., 1., 1.))
        def worker():
            processed.append(pairs.take(.2))
            entered.set()
            finish.wait(2.)
            processed.append(pairs.take(.2))
        thread = threading.Thread(target=worker)
        thread.daemon = True
        thread.start()
        try:
            self.assertTrue(entered.wait(2.))
            for stamp in range(2, 21):
                self.assertTrue(pairs.put(stamp, (stamp, stamp, stamp)))
        finally:
            finish.set()
            thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertEqual(processed, ['first', 20])
        self.assertEqual(pairs.overwritten, 18)
        self.assertIsNone(pairs.take(.001))

    def test_old_partial_or_invalid_pair_cannot_replace_newest(self):
        pairs = LatestPair()
        self.assertTrue(pairs.put('newest', (10., 10., 10.)))
        for stamps in ((9.,9.,9.), (11.,9.,11.), (11.,11.,10.),
                       (float('nan'),11.,11.), (0.,11.,11.), (11.,11.)):
            self.assertFalse(pairs.put('obsolete', stamps))
        self.assertEqual(pairs.take(), 'newest')
        self.assertEqual(pairs.overwritten, 0)
        self.assertFalse(pairs.put('repeated after take', (10.,10.,10.)))
        self.assertTrue(pairs.put('next', (11.,11.,11.)))
        self.assertEqual(pairs.take(), 'next')

    def test_enqueue_keeps_processing_state_untouched(self):
        # A slot delivery itself cannot claim a target or refresh ROS stamps:
        # message objects are preserved for the existing processor to validate.
        pairs = LatestPair()
        rgb, depth, info = object(), object(), object()
        message = (rgb, depth, info)
        pairs.put(message, (1.,1.01,1.))
        self.assertIs(pairs.take(), message)


if __name__ == '__main__':
    unittest.main()
