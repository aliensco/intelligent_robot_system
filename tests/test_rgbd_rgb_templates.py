import os
import sys
import unittest
import json
import math
import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)),
                              'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.rgb_templates import detect_templates, suppress_templates


class TemplateTests(unittest.TestCase):
    def test_captured_whole_base_outranks_near_tied_half_base_fragments(self):
        filename=os.path.join(os.path.dirname(__file__),'fixtures','rgbd_template_partial_peaks.json')
        with open(filename) as stream:
            fixture=json.load(stream)
        proposals=[]
        for item in fixture['proposals']:
            candidate=dict(bbox=item['bbox'],depth=item['depth'],_correlation=item['correlation'],
                           _size_error=abs(math.log(item['width_m']/fixture['reference_width_m'])))
            proposals.append(candidate)
        # The highest raw RGB correlation is the wrong 64 px half-base crop.
        self.assertEqual(max(proposals,key=lambda c:c['_correlation'])['bbox'][2],64)
        chosen,_=suppress_templates(proposals,np.zeros((480,640),np.uint8))
        self.assertEqual(len(chosen),1)
        self.assertEqual(chosen[0]['bbox'],[213,315,140,42])

    def scene(self, two=False):
        rgb = np.full((240,320,3),170,np.uint8)
        rows=np.mgrid[0:240,0:320][0]
        depth=np.full(rows.shape,np.nan,np.float32)
        below=rows>142
        depth[below]=.34*260/(rows[below]-119.5)
        pattern=np.random.RandomState(3).randint(0,255,(24,48)).astype(np.uint8)
        view=cv2.resize(pattern,(88,44))
        for x in ([25,205] if two else [110]):
            rgb[145:189,x:x+88]=view[:,:,None]
            depth[145:189,x:x+88]=1.
        cfg=dict(min_depth=.4,max_depth=4.,min_depth_samples=30,rgb_templates=[
            dict(pixels=pattern.tolist(),aspect=2.,width_m=88./260.,label='manual_test',
                 min_correlation=.9,verified_correlation=.95)])
        return rgb,depth,(260.,260.,159.5,119.5),cfg

    def test_fixed_template_uses_sparse_real_depth(self):
        rgb,d,camera,cfg=self.scene()
        d[145:189,110:198]=np.nan
        d[165:175,145:153]=1.
        candidates,_=detect_templates(rgb,d,camera,cfg)
        self.assertEqual(len(candidates),1)
        self.assertAlmostEqual(candidates[0]['depth'],1.)

    def test_rgb_match_without_depth_never_produces_target(self):
        rgb,d,camera,cfg=self.scene()
        d[145:189,110:198]=np.nan
        self.assertEqual(detect_templates(rgb,d,camera,cfg)[0],[])

    def test_two_real_identical_templates_remain_competing_targets(self):
        self.assertEqual(len(detect_templates(*self.scene(two=True))[0]),2)

    def test_dense_body_and_sparse_rgb_body_are_both_seen_in_one_frame(self):
        from rgbd_following_core.vision import detect
        from rgbd_following_core.tracking import Tracker
        rgb,d,camera,cfg=self.scene(two=True)
        cfg.update(proposal_mode='depth_body',
            body=dict(min_width_m=.13,max_width_m=.5,min_height_m=.075,
                      max_height_m=.20,max_support_height_m=.20),
            identity=dict(references=[],max_distance=.35,reacquire_distance=.22,
                          shape_ratio=2.,width_ratio=2.))
        d[145:189,205:293]=np.nan
        d[165:175,240:248]=1.
        candidates,_=detect(rgb,d,camera,cfg)
        self.assertEqual(len(candidates),2)
        self.assertEqual(candidates[0]['sources'],['depth_body','rgb_template'])
        self.assertEqual(candidates[0]['feature']['kind'],'depth_body_v1')
        tracker=Tracker()
        self.assertIsNone(tracker.update([c['point'] for c in candidates],1.,
                                        [c['identity_verified'] for c in candidates]))
        self.assertEqual(tracker.reason,'ambiguous_target')

    def test_no_floor_evidence_rejects_template(self):
        rgb,d,camera,cfg=self.scene()
        d[:]=1.
        self.assertEqual(detect_templates(rgb,d,camera,cfg)[0],[])

    def test_background_depth_through_black_base_is_rejected(self):
        rgb,d,camera,cfg=self.scene()
        d[145:189,110:198]=np.nan
        # The RGB logo still matches, but its remaining range is a patch of
        # background. Projecting this full base to that range sinks its bottom
        # below the measured floor and must not report a farther target.
        d[165:175,145:153]=1.45
        self.assertEqual(detect_templates(rgb,d,camera,cfg)[0],[])

    def test_near_base_larger_than_previous_224_pixel_limit(self):
        rgb=np.full((480,640,3),170,np.uint8)
        rows=np.mgrid[0:480,0:640][0]
        d=np.full(rows.shape,np.nan,np.float32)
        below=rows>270
        d[below]=.30*525/(rows[below]-239.5)
        pattern=np.random.RandomState(4).randint(0,255,(16,48)).astype(np.uint8)
        rgb[360:460,170:470]=cv2.resize(pattern,(300,100))[:,:,None]
        d[360:460,170:470]=.60
        cfg=dict(min_depth=.4,max_depth=4.,min_depth_samples=30,rgb_templates=[
            dict(pixels=pattern.tolist(),aspect=3.,width_m=300*.60/525,
                 label='near_base',min_correlation=.9,verified_correlation=.95)])
        candidates,_=detect_templates(rgb,d,(525.,525.,319.5,239.5),cfg)
        self.assertEqual(len(candidates),1)
        self.assertEqual(candidates[0]['bbox'],(170,360,300,100))
        self.assertAlmostEqual(candidates[0]['depth'],.60,places=6)


if __name__=='__main__':
    unittest.main()
