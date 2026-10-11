"""Structural evidence on physical scenes and an anonymized real depth crop.

Scenes are projected from dimensions in metres; object position, depth and
camera intrinsics vary independently of the structural classifier's bins.
These are recognition regressions, not navigation or identity acceptance.
"""
import copy
import json
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))


def physical_scene(kind='robot', distance=1.5, lateral=0., focal=420.):
    """Ray-project front surfaces, a distant wall and a measured flat floor."""
    camera = (focal, focal, 299.5, 199.5)
    plane = (np.array([0., -1., 0.]), .31)
    rows, cols = np.mgrid[:400, :600]
    horizontal = (cols-camera[2])*distance/focal-lateral
    height = plane[1]-(rows-camera[3])*distance/focal
    depth = np.full(rows.shape, 4., dtype=np.float32)
    below = rows > camera[3]
    depth[below] = np.minimum(depth[below],
                             plane[1]*focal/(rows[below]-camera[3]))

    def rectangle(xlo, xhi, hlo, hhi, z=distance):
        mask = ((horizontal >= xlo) & (horizontal <= xhi) &
                (height >= hlo) & (height <= hhi))
        depth[mask] = np.minimum(depth[mask], z)

    # Identical bottom body for all hypotheses; the classifier must use actual
    # upper geometry rather than assume the candidate's bottom is a robot.
    rectangle(-.16, .16, .025, .135)
    if kind == 'robot':
        for shelf in (.255, .415, .585):
            rectangle(-.17, .17, shelf-.012, shelf+.012)
        for post in (-.14, .14):
            rectangle(post-.009, post+.009, .12, .60)
    elif kind == 'cone':
        half_width = .15*(.36-height)/.30
        solid = ((height >= .06) & (height <= .36) &
                 (np.abs(horizontal) <= half_width))
        depth[solid] = distance
    elif kind == 'chair':
        rectangle(-.18, .18, .45, .49)
        for leg in (-.145, .145):
            rectangle(leg-.016, leg+.016, .03, .46)
    elif kind == 'box':
        rectangle(-.15, .15, .06, .38)
    elif kind == 'background_chair':
        # Background chair features cannot become same-depth upper support.
        rectangle(-.18, .18, .245, .275, z=distance+.80)
        for leg in (-.145, .145):
            rectangle(leg-.016, leg+.016, .135, .27, z=distance+.80)
    elif kind != 'base_only':
        raise ValueError(kind)

    left = int(round(camera[2]+(lateral-.16)*focal/distance))
    right = int(round(camera[2]+(lateral+.16)*focal/distance))
    top = int(round(camera[3]+(.31-.135)*focal/distance))
    bottom = int(round(camera[3]+(.31-.025)*focal/distance))
    candidate = dict(bbox=(left, top, right-left, bottom-top), depth=distance,
                     point=(lateral, .31-.08, distance), identity_verified=False,
                     identity_score=.13)
    return depth, camera, plane, candidate


def captured_scene():
    stem = os.path.join(ROOT, 'tests/fixtures/rgbd_upper_structure_capture')
    with open(stem+'.json') as stream:
        meta = json.load(stream)
    with np.load(stem+'.npz') as data:
        depth = data['depth'].copy()
    plane = (np.array(meta['floor_plane'][0]), meta['floor_plane'][1])
    return depth, meta['camera'], plane, meta


class StructureEvidenceTests(unittest.TestCase):
    def evidence(self, scene):
        from rgbd_following_core.structure import upper_structure_evidence
        return upper_structure_evidence(*scene, reference_width=.30)

    def test_physical_robot_is_supported_across_range_and_projection(self):
        for distance, lateral, focal in ((1.1, -.17, 390.),
                                          (1.6, .23, 470.),
                                          (2.1, -.12, 525.)):
            result = self.evidence(physical_scene('robot', distance, lateral, focal))
            self.assertEqual(result['state'], 'supported', result)

    def test_solid_tapered_obstacle_is_incompatible_at_multiple_distances(self):
        for distance, lateral, focal in ((.8, .12, 390.),
                                          (1.4, -.24, 470.),
                                          (1.9, .28, 525.)):
            result = self.evidence(physical_scene('cone', distance, lateral, focal))
            self.assertEqual(result['state'], 'incompatible', result)

    def test_chair_and_uniform_box_are_not_positive_robot_evidence(self):
        for kind in ('chair', 'box', 'background_chair'):
            result = self.evidence(physical_scene(kind))
            self.assertEqual(result['state'], 'unknown', (kind, result))

    def test_invalid_upper_depth_is_unknown_and_inputs_are_unchanged(self):
        for invalid in (np.nan, np.inf, 0.):
            scene = list(physical_scene('robot'))
            depth, camera, plane, candidate = scene
            last_upper_row = int(camera[3]+(.31-.14)*camera[1]/candidate['depth'])
            depth[:last_upper_row] = invalid
            before = depth.copy()
            original_candidate = copy.deepcopy(candidate)
            result = self.evidence(scene)
            self.assertEqual(result['state'], 'unknown', result)
            np.testing.assert_equal(depth, before)
            self.assertEqual(candidate, original_candidate)

    def test_cropped_upper_structure_is_unknown(self):
        depth, camera, plane, candidate = physical_scene('robot')
        start_row = candidate['bbox'][1]-1
        camera = list(camera)
        camera[3] -= start_row
        candidate = copy.deepcopy(candidate)
        x, y, w, h = candidate['bbox']
        candidate['bbox'] = (x, y-start_row, w, h)
        result = self.evidence((depth[start_row:].copy(), camera, plane, candidate))
        self.assertEqual(result['state'], 'unknown', result)

    def test_unmeasured_gap_cannot_support_a_visible_platform(self):
        scene = physical_scene('robot')
        depth, camera, plane, candidate = scene
        rows = np.mgrid[:depth.shape[0], :depth.shape[1]][0]
        height = plane[1]-(rows-camera[3])*candidate['depth']/camera[1]
        depth[(height > .145) & (height < .245)] = np.nan
        result = self.evidence(scene)
        self.assertEqual(result['state'], 'unknown', result)

    def test_two_real_robots_remain_distinct_and_identity_is_ambiguous(self):
        from rgbd_following_core.structure import filter_upper_structure
        from rgbd_following_core.tracking import Tracker
        left = physical_scene('robot', distance=1.5, lateral=-.24)
        right = physical_scene('robot', distance=1.5, lateral=.24)
        depth = np.minimum(left[0], right[0])
        candidates = filter_upper_structure([left[3], right[3]], depth,
                                             left[1], left[2], .30)
        self.assertEqual(len(candidates), 2, candidates)
        self.assertEqual([c['upper_structure']['state'] for c in candidates],
                         ['supported', 'supported'])
        self.assertFalse(any(c['identity_verified'] for c in candidates))
        tracker = Tracker()
        for stamp in (1., 1.1, 1.2, 1.3, 1.4):
            self.assertIsNone(tracker.update([c['point'] for c in candidates],
                                             stamp, [False, False]))
            self.assertEqual(tracker.reason, 'ambiguous_target')

    def test_two_partial_bases_share_one_actual_platform_without_identity_upgrade(self):
        from rgbd_following_core.structure import filter_upper_structure
        depth, camera, plane, original = physical_scene('robot')
        x, y, width, height = original['bbox']
        partials = []
        partial_width = 3*width//4
        # Two overlapping proposals of the same measured base, not disjoint
        # hypotheses outside the pre-existing spatial association boundary.
        for left in (x, x+width-partial_width):
            item = copy.deepcopy(original)
            item['bbox'] = (left, y, partial_width, height)
            item['point'] = ((left+partial_width/2.-camera[2])*item['depth']/camera[0],
                             original['point'][1], original['point'][2])
            partials.append(item)
        candidates = filter_upper_structure(partials, depth, camera, plane, .30)
        self.assertEqual(len(candidates), 1, candidates)
        self.assertEqual(candidates[0]['upper_structure']['state'], 'supported')
        self.assertFalse(candidates[0]['identity_verified'])
        self.assertAlmostEqual(candidates[0]['point'][0], 0., delta=.015)

    def test_center_correction_does_not_merge_originally_distant_bases(self):
        from rgbd_following_core.structure import filter_upper_structure
        depth, camera, plane, original = physical_scene('robot')
        x, y, width, height = original['bbox']
        partials = []
        for left in (x, x+width//2):
            item = copy.deepcopy(original)
            item['bbox'] = (left, y, width//2, height)
            item['point'] = ((left+width/4.-camera[2])*item['depth']/camera[0],
                             original['point'][1], original['point'][2])
            partials.append(item)
        candidates = filter_upper_structure(partials, depth, camera, plane, .30)
        self.assertEqual(len(candidates), 2, candidates)
        self.assertTrue(all(c['upper_structure']['state'] == 'supported'
                            for c in candidates))

    def test_near_base_cannot_borrow_distant_robots_measured_platform(self):
        from rgbd_following_core.structure import filter_upper_structure
        near = physical_scene('base_only', distance=1.0)
        far = physical_scene('robot', distance=1.7)
        depth = np.minimum(near[0], far[0])
        candidates = filter_upper_structure([near[3], far[3]], depth,
                                             near[1], near[2], .30)
        self.assertEqual(len(candidates), 2, candidates)
        self.assertEqual(candidates[0]['upper_structure']['state'], 'unknown')
        self.assertEqual(candidates[1]['upper_structure']['state'], 'supported')
        self.assertEqual(candidates[0]['point'], near[3]['point'])

    def test_near_background_shelf_cannot_move_base_sideways(self):
        from rgbd_following_core.structure import filter_upper_structure
        depth, camera, plane, candidate = physical_scene('base_only',
                                                        distance=1.5, lateral=.5)
        rows, cols = np.mgrid[:depth.shape[0], :depth.shape[1]]
        shelf_depth = 1.68
        horizontal = (cols-camera[2])*shelf_depth/camera[0]
        height = plane[1]-(rows-camera[3])*shelf_depth/camera[1]
        shelf = ((np.abs(horizontal-.68) < .15) &
                 (height >= .243) & (height <= .267))
        depth[shelf] = shelf_depth
        result = filter_upper_structure([candidate], depth, camera, plane, .30)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['upper_structure']['state'], 'unknown', result)
        self.assertEqual(result[0]['point'], candidate['point'])

    def test_thin_near_rod_cannot_borrow_background_shelf_width(self):
        from rgbd_following_core.structure import filter_upper_structure
        depth, camera, plane, candidate = physical_scene('base_only', distance=1.5)
        rows, cols = np.mgrid[:depth.shape[0], :depth.shape[1]]
        horizontal = (cols-camera[2])*candidate['depth']/camera[0]
        height = plane[1]-(rows-camera[3])*candidate['depth']/camera[1]
        shelf = ((np.abs(horizontal) < .15) &
                 (height >= .245) & (height <= .267))
        depth[shelf] = 1.68
        depth[shelf & (np.abs(horizontal) < .01)] = candidate['depth']
        result = filter_upper_structure([candidate], depth, camera, plane, .30)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['upper_structure']['state'], 'unknown', result)
        self.assertEqual(result[0]['point'], candidate['point'])

    def test_finite_out_of_range_background_is_not_an_observed_gap(self):
        scene = physical_scene('robot')
        depth, camera, plane, candidate = scene
        depth[depth > candidate['depth']+.25] = 8.
        result = self.evidence(scene)
        self.assertEqual(result['state'], 'unknown', result)

    def test_background_at_configured_maximum_is_still_observed(self):
        from rgbd_following_core.structure import upper_structure_evidence
        scene = physical_scene('robot')
        depth, camera, plane, candidate = scene
        depth[depth > candidate['depth']+.25] = 4.
        result = upper_structure_evidence(depth, camera, plane, candidate, .30,
                                           min_depth=.4, max_depth=4.)
        self.assertEqual(result['state'], 'supported', result)

    def test_candidate_support_below_configured_minimum_is_unknown(self):
        from rgbd_following_core.structure import upper_structure_evidence
        scene = physical_scene('robot', distance=1.5)
        result = upper_structure_evidence(*scene, reference_width=.30,
                                           min_depth=1.6, max_depth=4.)
        self.assertEqual(result['state'], 'unknown', result)

    def test_real_captured_cone_is_rejected_without_discarding_partial_robot(self):
        depth, camera, plane, meta = captured_scene()
        results = {}
        for candidate in meta['proposals']:
            results[candidate['label']] = self.evidence((depth, camera, plane, candidate))
        self.assertEqual(results['blue_cone_base']['state'], 'incompatible', results)
        self.assertNotEqual(results['front_turtlebot_partial_base']['state'],
                            'incompatible', results)

    def test_real_evidence_is_invariant_to_pixel_origin_shift(self):
        depth, camera, plane, meta = captured_scene()
        shifted = np.pad(depth, ((17, 11), (23, 19)),
                         mode='constant', constant_values=np.nan)
        shifted_camera = list(camera)
        shifted_camera[2] += 23
        shifted_camera[3] += 17
        for candidate in meta['proposals']:
            expected = self.evidence((depth, camera, plane, candidate))
            moved = copy.deepcopy(candidate)
            x, y, w, h = moved['bbox']
            moved['bbox'] = (x+23, y+17, w, h)
            result = self.evidence((shifted, shifted_camera, plane, moved))
            self.assertEqual(result['state'], expected['state'], (expected, result))


if __name__ == '__main__':
    unittest.main()
