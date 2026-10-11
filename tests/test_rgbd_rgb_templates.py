import os
import sys
import unittest
import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)),
                              'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.rgb_templates import detect_templates


class TemplateTests(unittest.TestCase):
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

    def test_no_floor_evidence_rejects_template(self):
        rgb,d,camera,cfg=self.scene()
        d[:]=1.
        self.assertEqual(detect_templates(rgb,d,camera,cfg)[0],[])


if __name__=='__main__':
    unittest.main()
