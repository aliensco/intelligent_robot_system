import copy
import os
import sys
import unittest
import numpy as np
import cv2
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.vision import detect
from rgbd_following_core.settings import validate


class BodyTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/config/follow.yaml')) as f:
            self.config = yaml.safe_load(f)
        self.config['perception']['proposal_mode'] = 'depth_body'
        self.camera = (260., 260., 159.5, 119.5)

    def scene(self, centers=(160,), color=None):
        rgb = np.full((240, 320, 3), 170, dtype=np.uint8)
        y = np.mgrid[0:240, 0:320][0]
        depth = np.full(y.shape, np.nan, dtype=np.float32)
        below = y > 142
        depth[below] = .34*260/(y[below]-119.5)
        for center in centers:
            depth[120:200, center-24:center+24] = 1.
            rgb[120:160, center-24:center+24] = 230 if color is None else color
            rgb[160:200, center-24:center+24] = 35 if color is None else color
        return rgb, depth

    def candidates(self, rgb, depth):
        return detect(rgb, depth, self.camera, self.config['perception'])[0]

    def enroll(self):
        candidates = self.candidates(*self.scene())
        self.assertEqual(len(candidates), 1)
        self.config['perception']['identity']['references'] = [candidates[0]['feature']]
        validate(self.config)

    def test_black_white_body_separated_from_gray_floor(self):
        candidates = self.candidates(*self.scene())
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0]['depth'], 1.)
        self.assertEqual(candidates[0]['feature']['kind'], 'depth_body_v1')

    def test_duplicate_depth_slices_do_not_create_ambiguity(self):
        self.enroll()
        self.assertEqual(len(self.candidates(*self.scene())), 1)

    def test_nested_partial_body_at_different_depth_is_one_target(self):
        rgb, depth = self.scene()
        self.config['perception']['body']['min_height_m'] = .06
        self.config['perception']['body']['min_width_m'] = .10
        depth[176:200, 136:184] = 1.16
        self.assertEqual(len(self.candidates(rgb, depth)), 1)

    def test_bad_larger_window_cannot_suppress_matching_overlap(self):
        self.enroll()
        # Simulate appearance contamination in one overlapping depth window.
        # Detection must still evaluate the other window before suppression.
        import rgbd_following_core.body as body
        original = body.match_reference
        calls = []
        def score(*args):
            calls.append(args[0])
            return .9 if len(calls) == 1 else .1
        body.match_reference = score
        try:
            candidates = self.candidates(*self.scene())
        finally:
            body.match_reference = original
        self.assertGreater(len(calls), 1)
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0]['identity_score'], .1)
        self.assertNotIn('_support', candidates[0])

    def test_two_similar_objects_remain_ambiguous(self):
        self.enroll()
        self.assertEqual(len(self.candidates(*self.scene((80, 240)))), 2)

    def test_same_shape_different_appearance_is_rejected(self):
        self.enroll()
        self.assertEqual(self.candidates(*self.scene(color=(0, 0, 220))), [])

    def test_small_exposure_change_keeps_same_body(self):
        self.enroll()
        rgb, depth = self.scene()
        rgb[120:200, 136:184] = np.clip(
            rgb[120:200, 136:184].astype(int)+12, 0, 255).astype(np.uint8)
        self.assertEqual(len(self.candidates(rgb, depth)), 1)

    def test_brightness_smoothing_preserves_color_and_quadrant_mass(self):
        from rgbd_following_core.appearance import smooth_body_brightness
        source = [0.]*144
        source[0], source[11], source[12], source[143] = .2, .3, .1, .4
        smoothed = smooth_body_brightness(source)
        self.assertAlmostEqual(sum(smoothed), sum(source))
        self.assertEqual(smoothed[12], .1)
        self.assertEqual(smoothed[143], .4)
        self.assertEqual(sum(smoothed[36:108]), 0.)

    def test_no_floor_never_falls_back_to_color(self):
        rgb, depth = self.scene()
        depth[:] = 1.
        self.assertEqual(self.candidates(rgb, depth), [])

    def test_invalid_depth_returns_no_target(self):
        rgb, depth = self.scene()
        depth[:] = np.nan
        self.assertEqual(self.candidates(rgb, depth), [])

    def test_lower_base_band_excludes_upper_structure_without_changing_depth(self):
        rgb, depth = self.scene()
        before = depth.copy()
        body = self.config['perception']['body']
        body.update(min_height_m=.08, max_height_m=.20, max_support_height_m=.17)
        validate(self.config)
        candidates, mask = detect(rgb, depth, self.camera, self.config['perception'])
        self.assertEqual(len(candidates), 1)
        self.assertLess(candidates[0]['feature']['height_m'], .17)
        self.assertFalse(mask[:160].any())
        np.testing.assert_equal(depth, before)

    def test_invalid_recognition_support_band_is_rejected(self):
        for height in (0., .03, .9, float('nan')):
            self.config['perception']['body']['max_support_height_m'] = height
            with self.assertRaises(ValueError):
                validate(self.config)

    def test_floating_body_is_rejected(self):
        rgb, depth = self.scene(())
        depth[30:110, 136:184] = 1.
        self.assertEqual(self.candidates(rgb, depth), [])

    def test_legacy_color_references_cannot_enable_body_mode(self):
        self.enroll()
        del self.config['perception']['identity']['references'][0]['kind']
        with self.assertRaises(ValueError):
            validate(self.config)

    def test_invalid_silhouette_is_rejected(self):
        self.enroll()
        self.config['perception']['identity']['references'][0]['silhouette'] = [0.]
        with self.assertRaises(ValueError):
            validate(self.config)

    def test_floor_can_be_minority_with_distributed_support(self):
        self.enroll()
        rgb, depth = self.scene()
        # Obstacles hide most central floor pixels, but actual floor remains
        # visible on both sides and along the bottom at multiple depths.
        depth[143:236, 60:260] = 1.5
        rgb[143:236, 60:260] = (0, 0, 220)
        depth[120:200, 136:184] = 1.
        rgb[120:160, 136:184] = 230
        rgb[160:200, 136:184] = 35
        candidates = self.candidates(rgb, depth)
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0]['depth'], 1.)

    def test_narrow_floor_strip_is_not_enough_evidence(self):
        rgb, depth = self.scene()
        depth[:, :136] = np.nan
        depth[:, 184:] = np.nan
        self.assertEqual(self.candidates(rgb, depth), [])


if __name__ == '__main__':
    unittest.main()
