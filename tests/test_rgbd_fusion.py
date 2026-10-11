"""RGB and depth alternatives must observe the full scene, without double IDs."""
import os
import sys
import unittest
import json
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)),
                              'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.vision import detect, fuse_body_templates
from rgbd_following_core.tracking import Tracker


def candidate(x=100, depth=1., verified=True):
    return dict(bbox=(x,100,80,40),point=((x-100)/260.,.1,depth),depth=depth,
                identity_verified=verified,identity_score=.1)


class FusionTests(unittest.TestCase):
    def test_same_body_merges_once_and_keeps_measured_rgb_position(self):
        body,visual=candidate(),candidate(102)
        body['point']=(.04,.1,1.)
        merged=fuse_body_templates([body],[visual])
        self.assertEqual(len(merged),1)
        self.assertEqual(merged[0]['point'],visual['point'])
        self.assertEqual(merged[0]['sources'],['depth_body','rgb_template'])

    def test_independent_robots_keep_tracker_ambiguity(self):
        merged=fuse_body_templates([candidate(30)],[candidate(180)])
        self.assertEqual(len(merged),2)
        tracker=Tracker()
        self.assertIsNone(tracker.update([p['point'] for p in merged],1.))
        self.assertEqual(tracker.reason,'ambiguous_target')

    def test_overlapping_depth_planes_remain_independent(self):
        self.assertEqual(len(fuse_body_templates([candidate()],[candidate(depth=1.3)])),2)

    def test_many_to_one_overlap_does_not_hide_a_second_robot(self):
        self.assertEqual(len(fuse_body_templates([candidate(98),candidate(102)],
                                                [candidate()])),3)

    def captured_partial_depth(self):
        filename=os.path.join(os.path.dirname(__file__),'fixtures','rgbd_partial_body_fusion.json')
        with open(filename) as stream:
            fixture=json.load(stream)
        samples=[]
        for value,count in fixture['depth_runs_mm']:
            samples.extend([value/1000. if value else float('nan')]*count)
        patch=np.asarray(samples,np.float32).reshape(fixture['crop_shape'])
        depth=np.full((480,640),np.nan,np.float32)
        x,y=fixture['crop_origin']
        depth[y:y+patch.shape[0],x:x+patch.shape[1]]=patch
        return fixture,depth

    def test_captured_black_base_holes_merge_using_shared_real_depth(self):
        fixture,depth=self.captured_partial_depth()
        body,visual=fixture['body'],fixture['visual']
        # This exact field frame has only 57.1% rectangle overlap, but 73.0%
        # shared measured support. The original fusion returned two targets.
        self.assertEqual(len(fuse_body_templates([body],[visual])),2)
        result=fuse_body_templates([body],[visual],depth)
        self.assertEqual(len(result),1)
        self.assertEqual(result[0]['point'],visual['point'])

    def test_partial_overlap_without_measured_support_stays_separate(self):
        fixture,depth=self.captured_partial_depth()
        depth[:]=np.nan
        self.assertEqual(len(fuse_body_templates([fixture['body']],[fixture['visual']],depth)),2)

    def test_shared_patch_does_not_expand_spatial_or_depth_association(self):
        fixture,depth=self.captured_partial_depth()
        for field,value in [('point',[.2,.22,1.287]),('depth',1.6)]:
            visual=dict(fixture['visual'])
            visual[field]=value
            self.assertEqual(len(fuse_body_templates([fixture['body']],[visual],depth)),2)

    def test_rgb_search_runs_even_when_depth_body_has_another_candidate(self):
        import rgbd_following_core.body as body
        import rgbd_following_core.rgb_templates as visual
        original_body,original_rgb=body.detect_body,visual.detect_templates
        mask=np.zeros((240,320),np.uint8)
        calls=[]
        def visual_candidates(*args):
            calls.append(True)
            return [candidate(180)],mask
        body.detect_body=lambda *args: ([candidate(30)],mask)
        visual.detect_templates=visual_candidates
        try:
            result,_=detect(np.zeros((240,320,3),np.uint8),mask.astype(np.float32),
                            (260.,260.,159.5,119.5),
                            dict(proposal_mode='depth_body',rgb_templates=[{}],min_depth_samples=30))
        finally:
            body.detect_body,visual.detect_templates=original_body,original_rgb
        self.assertEqual(calls,[True])
        self.assertEqual(len(result),2)


if __name__=='__main__':
    unittest.main()
