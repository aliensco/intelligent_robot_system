import os
import sys
import unittest
import json
import math
import copy
import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)),
                              'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.rgb_templates import detect_templates, suppress_templates


class TemplateTests(unittest.TestCase):
    def continuous_base_fragments(self, bridge=1.5):
        """Independent rays through two small patches of a low robot body.

        Only the strip between the two patch centers changes across cases. This
        distinguishes a measured continuous body from a background/invalid
        connection without changing the RGB proposal geometry or confidence.
        """
        camera=(500.,500.,50.,-30.)
        plane=(np.array((0.,-1.,0.)),.30)
        config=dict(min_depth=.4,max_depth=4.,min_depth_samples=30,
                    body=dict(max_height_m=.20))
        depth=np.full((65,90),np.nan,np.float32)
        depth[15:50,20:60]=1.5
        depth[30:35,20:60]=bridge
        candidates=[]
        for rank,box in enumerate(((20,15,40,23),(20,27,40,23))):
            x,y,w,h=box
            values=depth[y:y+h,x:x+w]
            candidates.append(dict(bbox=box,depth=1.5,
                point=((x+w/2.-camera[2])*1.5/camera[0],
                       (y+h/2.-camera[3])*1.5/camera[1],1.5),
                identity_verified=False,_correlation=.92-.01*rank,
                _size_error=0.,_depth_support=(np.isfinite(values) &
                    (np.abs(values-1.5)<=.10))))
        return candidates,(depth,camera,plane,config)

    def test_connected_real_surface_merges_offset_base_fragments(self):
        from rgbd_following_core.rgb_templates import connected_template_support
        proposals,context=self.continuous_base_fragments()
        before=context[0].copy()
        self.assertTrue(connected_template_support(proposals[0],proposals[1],*context))
        chosen,_=suppress_templates(proposals,np.zeros(before.shape,np.uint8),
                                    surface_context=context)
        self.assertEqual(len(chosen),1)
        self.assertFalse(chosen[0]['identity_verified'])
        self.assertNotIn('_depth_support',chosen[0])
        np.testing.assert_equal(context[0],before)

    def test_missing_and_background_depth_cannot_connect_separate_base_surfaces(self):
        from rgbd_following_core.rgb_templates import connected_template_support
        for bridge in (np.nan,np.inf,0.,1.68,1.55):
            proposals,context=self.continuous_base_fragments(bridge)
            # 1.55 m is within each proposal's broad depth support band, but
            # its 5 cm edge discontinuity is a separate observed surface.
            self.assertFalse(connected_template_support(proposals[0],proposals[1],
                                                        *context),str(bridge))
            chosen,_=suppress_templates(proposals,np.zeros(context[0].shape,np.uint8),
                                        surface_context=context)
            self.assertEqual(len(chosen),2,str(bridge))

    def test_adjacent_same_depth_robots_remain_ambiguous_with_surface_context(self):
        from rgbd_following_core.rgb_templates import connected_template_support
        from rgbd_following_core.tracking import Tracker
        proposals,context=self.continuous_base_fragments()
        depth,camera,plane,config=context
        depth[:]=np.nan
        proposals[0].update(bbox=(10,25,30,20),point=(-.075,.195,1.5),
                            _depth_support=np.ones((20,30),bool))
        proposals[1].update(bbox=(42,25,30,20),point=(.021,.195,1.5),
                            _depth_support=np.ones((20,30),bool))
        depth[25:45,10:40]=1.5
        depth[25:45,42:72]=1.5
        self.assertFalse(connected_template_support(proposals[0],proposals[1],*context))
        chosen,_=suppress_templates(proposals,np.zeros(depth.shape,np.uint8),
                                    surface_context=context)
        self.assertEqual(len(chosen),2)
        tracker=Tracker()
        self.assertIsNone(tracker.update([c['point'] for c in chosen],1.,[False,False]))
        self.assertEqual(tracker.reason,'ambiguous_target')

    def test_continuous_depth_chain_cannot_merge_incompatible_end_targets(self):
        from rgbd_following_core.rgb_templates import connected_template_support
        camera=(500.,500.,50.,-45.)
        plane=(np.array((0.,-1.,0.)),.30)
        config=dict(min_depth=.4,max_depth=4.,min_depth_samples=30,
                    body=dict(max_height_m=.20))
        depth=np.full((80,90),np.nan,np.float32)
        depth[15:65,20:60]=1.08
        depth[15:27,20:60]=1.
        depth[27:32,20:60]=np.linspace(1.,1.08,5)[:,None]
        depth[45:50,20:60]=np.linspace(1.08,1.16,5)[:,None]
        depth[50:65,20:60]=1.16
        candidates=[]
        for y,z,score in ((15,1.,.94),(30,1.08,.95),(45,1.16,.93)):
            values=depth[y:y+20,20:60]
            candidates.append(dict(bbox=(20,y,40,20),depth=z,
                point=(-10*z/500.,(y+10+45)*z/500.,z),
                identity_verified=False,_correlation=score,_size_error=0.,
                _depth_support=np.isfinite(values)&(np.abs(values-z)<=.10)))
        context=(depth,camera,plane,config)
        a,b,c=candidates
        self.assertTrue(connected_template_support(a,b,*context))
        self.assertTrue(connected_template_support(b,c,*context))
        self.assertFalse(connected_template_support(a,c,*context))
        chosen,_=suppress_templates(candidates,np.zeros(depth.shape,np.uint8),
                                    surface_context=context)
        self.assertEqual(len(chosen),2)
        self.assertIn(c['bbox'],[item['bbox'] for item in chosen])

    def partial_occlusion_fixture(self, erase_shared=False):
        stem=os.path.join(os.path.dirname(__file__),'fixtures','rgbd_partial_occlusion_support')
        with open(stem+'.json') as stream:
            fixture=json.load(stream)
        with np.load(stem+'.npz') as data:
            depth=data['depth'].copy()
        if erase_shared:
            a,b=fixture['proposals']
            x0=max(a['bbox'][0],b['bbox'][0])
            y0=max(a['bbox'][1],b['bbox'][1])
            x1=min(a['bbox'][0]+a['bbox'][2],b['bbox'][0]+b['bbox'][2])
            y1=min(a['bbox'][1]+a['bbox'][3],b['bbox'][1]+b['bbox'][3])
            depth[y0:y1,x0:x1]=np.nan
        fx,fy,cx,cy=fixture['camera']
        normal,offset=fixture['floor_plane']
        proposals=[]
        for item in fixture['proposals']:
            candidate=copy.deepcopy(item)
            x,y,w,h=candidate['bbox']
            values=depth[y:y+h,x:x+w]
            v,u=np.mgrid[y:y+h,x:x+w]
            finite=np.isfinite(values)
            z=np.where(finite,values,0.)
            ground_height=((u-cx)*z/fx*normal[0]+(v-cy)*z/fy*normal[1]+
                           z*normal[2]+offset)
            # Recreate the detector's documented measured-base support from
            # the preserved real depth, not a fabricated positive merge mask.
            candidate['_depth_support']=(finite & (z>=.4) & (z<=4.) &
                (ground_height>.035) & (ground_height<.20) &
                (np.abs(z-candidate['depth'])<=.10))
            candidate['_correlation']=candidate.pop('correlation')
            candidate['_size_error']=candidate.pop('size_error')
            proposals.append(candidate)
        return proposals,depth

    def test_occluded_real_base_duplicate_uses_shared_measured_depth(self):
        proposals,depth=self.partial_occlusion_fixture()
        before=depth.copy()
        self.assertEqual(len(proposals),2)
        chosen,_=suppress_templates(proposals,np.zeros(depth.shape,np.uint8))
        self.assertEqual(len(chosen),1)
        self.assertEqual(chosen[0]['bbox'],[17,16,56,17])
        self.assertNotIn('_depth_support',chosen[0])
        self.assertFalse(chosen[0]['identity_verified'])
        np.testing.assert_equal(depth,before)

    def test_invalid_overlap_cannot_prove_duplicate_occluded_bases(self):
        proposals,depth=self.partial_occlusion_fixture(erase_shared=True)
        chosen,_=suppress_templates(proposals,np.zeros(depth.shape,np.uint8))
        self.assertEqual(len(chosen),2)
        self.assertTrue(all('_depth_support' not in c for c in chosen))

    def test_near_cone_is_not_merged_into_a_far_occluded_robot(self):
        proposals,depth=self.partial_occlusion_fixture()
        proposals[0]['depth']=.80
        proposals[0]['point'][2]=.80
        # Even coincident image support cannot identify two measured ranges
        # as one object. This changes geometry, not the suppression threshold.
        chosen,_=suppress_templates(proposals,np.zeros(depth.shape,np.uint8))
        self.assertEqual(len(chosen),2)

    def test_partly_overlapping_same_range_robots_with_separate_support_stay_two(self):
        from rgbd_following_core.tracking import Tracker
        first=dict(bbox=(20,20,40,20),depth=1.5,point=(-.04,.15,1.5),
                   identity_verified=True,_correlation=.92,_size_error=0.,
                   _depth_support=np.zeros((20,40),bool))
        second=dict(bbox=(40,20,40,20),depth=1.5,point=(.04,.15,1.5),
                    identity_verified=True,_correlation=.91,_size_error=0.,
                    _depth_support=np.zeros((20,40),bool))
        # Both boxes include the intervening image region, but their actual
        # observed surfaces occupy different rays at this identical distance.
        first['_depth_support'][:,:19]=True
        second['_depth_support'][:,21:]=True
        chosen,_=suppress_templates([first,second],np.zeros((60,100),np.uint8))
        self.assertEqual(len(chosen),2)
        tracker=Tracker()
        self.assertIsNone(tracker.update([c['point'] for c in chosen],1.,[True,True]))
        self.assertEqual(tracker.reason,'ambiguous_target')

    def test_shared_support_chain_does_not_collapse_two_distinct_objects(self):
        a=dict(bbox=(20,10,40,20),depth=1.5,point=(0.,.10,1.5),
               identity_verified=False,_correlation=.94,_size_error=0.,
               _depth_support=np.zeros((20,40),bool))
        b=dict(bbox=(20,20,40,20),depth=1.5,point=(0.,.14,1.5),
               identity_verified=False,_correlation=.95,_size_error=0.,
               _depth_support=np.ones((20,40),bool))
        c=dict(bbox=(20,30,40,20),depth=1.5,point=(0.,.18,1.5),
               identity_verified=False,_correlation=.93,_size_error=0.,
               _depth_support=np.zeros((20,40),bool))
        a['_depth_support'][10:]=True
        c['_depth_support'][:10]=True
        # B overlaps both actual surfaces. A and C have no shared measured ray;
        # choosing B first must not turn that overlap into a transitive merge.
        chosen,_=suppress_templates([a,b,c],np.zeros((65,80),np.uint8))
        self.assertEqual(len(chosen),2)
        self.assertIn(c['bbox'],[item['bbox'] for item in chosen])

    def test_geometric_overlap_cannot_bypass_an_existing_measured_support_group(self):
        a=dict(bbox=(20,10,40,20),depth=1.5,point=(0.,.10,1.5),
               _correlation=.94,_size_error=0.,_depth_support=np.zeros((20,40),bool))
        b=dict(bbox=(20,20,40,20),depth=1.5,point=(0.,.14,1.5),
               _correlation=.95,_size_error=0.,_depth_support=np.zeros((20,40),bool))
        c=dict(bbox=(20,24,40,20),depth=1.5,point=(0.,.18,1.5),
               _correlation=.93,_size_error=0.,_depth_support=np.zeros((20,40),bool))
        a['_depth_support'][10:]=True
        b['_depth_support'][:10]=True
        b['_depth_support'][14:]=True
        c['_depth_support'][10:]=True
        # The winning B shares A's upper surface and geometrically overlaps
        # C's lower surface. Neither kind of evidence relates A directly to C.
        chosen,_=suppress_templates([a,b,c],np.zeros((65,80),np.uint8))
        self.assertEqual(len(chosen),2)
        self.assertIn(c['bbox'],[item['bbox'] for item in chosen])

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

    def test_scale_prefilter_preserves_split_samples_around_valid_median(self):
        from rgbd_following_core.rgb_templates import scale_has_depth_support
        # The median is exactly 1 m, but no individual measurement is 1 m.
        # These real samples all pass the detector's +/- 10 cm depth support.
        samples=np.array([.91]*20+[1.09]*20,np.float32)
        cfg=dict(min_depth=.4,max_depth=4.,min_depth_samples=30)
        self.assertTrue(scale_has_depth_support(samples,150,.3,(1.,1.),500.,cfg))
        self.assertFalse(scale_has_depth_support(samples,60,.3,(.95,1.05),500.,cfg))

    def test_scale_prefilter_matches_full_search_with_two_independent_targets(self):
        import rgbd_following_core.rgb_templates as templates
        scene=self.scene(two=True)
        pruned,_=templates.detect_templates(*scene)
        original=templates.scale_has_depth_support
        templates.scale_has_depth_support=lambda *args: True
        try:
            unpruned,_=templates.detect_templates(*scene)
        finally:
            templates.scale_has_depth_support=original
        self.assertEqual(len(pruned),2)
        self.assertEqual([(c['bbox'],c['point']) for c in pruned],
                         [(c['bbox'],c['point']) for c in unpruned])


if __name__=='__main__':
    unittest.main()
