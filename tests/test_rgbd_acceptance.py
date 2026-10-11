"""Evidence regressions: a short motion/old cloud must not become a full PASS."""
from __future__ import division
import copy
import json
import os
import shutil
import sys
import tempfile
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import evaluate_rgbd_acceptance as acceptance


class Record(object):
    def __init__(self, **values):
        self.__dict__.update(values)


class Stamp(object):
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.trial = dict(scenario='straight_follow', terminal_reason='goal_reached',
                          endpoint=dict(xy=[.54, 0], tolerance_m=.1),
                          verdict=dict((k, True) for k in acceptance.VERDICTS+('route_completed',)))
        self.rows = []
        for i in range(61):
            t = 100+i*.1
            row = dict(kind='sample', t=t, target_valid=True, target_valid_stamp=t,
                       target_stamp=t, xy=[min(i, 54)*.01, 0], speed=.1 if i < 54 else 0.,
                       angular_speed=0., target_distance_m=.6, costmap_clearance_m=.04,
                       bumper=0, cliff=0, wheel_drop=0)
            row.update(dict((k+'_stamp', t) for k in ('odom', 'rgb', 'depth', 'grid', 'sensor')))
            self.rows.append(row)

    def test_complete_run_requires_all_evidence(self):
        result = acceptance.summarize(self.rows, self.trial)
        self.assertEqual(result['status'], 'PASS', result)
        self.assertAlmostEqual(result['metrics']['travel_m'], .54)
        self.assertEqual(result['metrics']['gap_error_p95_m'], 0.)

    def test_centimetre_trial_and_budget_stop_are_incomplete(self):
        for row in self.rows:
            row['xy'][0] *= .03
        self.trial['terminal_reason'] = 'travel_budget'
        result = acceptance.summarize(self.rows, self.trial)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('insufficient_duration_or_travel', result['missing'])
        self.assertIn('not_a_completed_run', result['missing'])
        self.assertIn('expected_endpoint_not_reached', result['missing'])

    def test_fresh_pointcloud_does_not_substitute_rgb_and_depth(self):
        for row in self.rows:
            row['rgb_stamp'] = row['depth_stamp'] = 100.
        result = acceptance.summarize(self.rows, self.trial)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('insufficient_fresh_rgb', result['missing'])
        self.assertIn('insufficient_fresh_depth', result['missing'])

    def test_missing_obstacle_clearance_never_passes_obstacle_trial(self):
        self.trial.update(scenario='multiple_obstacles', observed_obstacle_count=1)
        for row in self.rows:
            row['costmap_clearance_m'] = None
        result = acceptance.summarize(self.rows, self.trial)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('no_measured_obstacle_clearance', result['missing'])
        self.assertIn('multiple_obstacles_not_documented', result['missing'])

    def test_contact_or_minimum_distance_or_map_overlap_fails(self):
        for name, value in [('bumper', 1), ('target_distance_m', .39), ('costmap_clearance_m', -.001)]:
            rows = copy.deepcopy(self.rows)
            rows[25][name] = value
            self.assertEqual(acceptance.summarize(rows, self.trial)['status'], 'FAIL', name)

    def test_loss_measured_by_time_and_stale_validity_cannot_hide_it(self):
        for row in self.rows[10:50]:
            row['target_valid_stamp'] = 90.
        result = acceptance.summarize(self.rows, self.trial)
        self.assertEqual(result['status'], 'FAIL')
        self.assertAlmostEqual(result['metrics']['longest_target_loss_s'], 4.)
        self.assertAlmostEqual(result['metrics']['visible_fraction'], 1./3.)

    def test_nonfinite_or_clock_reset_evidence_is_incomplete(self):
        self.rows[2]['target_distance_m'] = float('nan')
        result = acceptance.summarize(self.rows, self.trial)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('nonfinite_measurements', result['missing'])
        self.rows[2]['target_distance_m'] = .6
        self.rows[2]['t'] = 99.
        self.assertIn('sampling_gap_or_clock_reset', acceptance.summarize(self.rows, self.trial)['missing'])

    def test_moving_or_unconfirmed_finish_is_incomplete(self):
        self.rows[-1]['speed'] = .05
        self.trial['verdict'].pop('route_completed')
        result = acceptance.summarize(self.rows, self.trial)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('no_measured_stopped_finish', result['missing'])

    def test_suite_requires_unmeasured_scenarios_and_independent_repeats(self):
        folder = tempfile.mkdtemp()
        try:
            path = os.path.join(folder, 'straight.jsonl')
            with open(path, 'w') as target:
                for row in self.rows:
                    target.write(json.dumps(row)+'\n')
            trial = dict(self.trial, log='straight.jsonl')
            report = acceptance.evaluate(dict(repeats=1, trials=[trial]), folder)
            self.assertEqual(report['scenarios']['straight_follow'], 'PASS')
            self.assertEqual(report['scenarios']['multiple_obstacles'], 'INCOMPLETE')
            self.assertEqual(report['status'], 'INCOMPLETE')
            with self.assertRaises(ValueError):
                acceptance.evaluate(dict(trials=[trial, trial]), folder)
            with open(os.path.join(folder, 'motion.log'), 'w') as target:
                target.write(json.dumps(dict(final_reason='duration_limit'))+'\n')
            trial['motion_log'] = 'motion.log'
            report = acceptance.evaluate(dict(repeats=1, trials=[trial]), folder)
            self.assertEqual(report['trials'][0]['metrics']['terminal_reason'], 'duration_limit')
            self.assertEqual(report['trials'][0]['status'], 'INCOMPLETE')
        finally:
            shutil.rmtree(folder)

    def test_bag_uses_checked_rgbd_source_stamps_not_status_arrival(self):
        module = types.ModuleType('rosbag')
        records = []
        ns = '/method3_motion'
        for row in self.rows:
            t = row['t']
            header = Record(stamp=Stamp(t), frame_id='odom')
            position = Record(x=row['xy'][0], y=0., z=0.)
            odom = Record(header=header, pose=Record(pose=Record(position=position)),
                          twist=Record(twist=Record(linear=Record(x=row['speed']), angular=Record(z=0.))))
            sensor = Record(header=header, bumper=0, cliff=0, wheel_drop=0)
            target = Record(header=header, pose=Record(position=Record(x=position.x+.6, y=0.)))
            grid = Record(header=header, data=[], info=Record(width=1, resolution=.05,
                origin=Record(position=Record(x=0., y=0.), orientation=Record(x=0., y=0., z=0., w=1.))))
            perception = Record(data=json.dumps(dict(rgb_stamp=t, depth_stamp=t)))
            messages = [(ns+'/perception_status', perception), ('/mobile_base/sensors/core', sensor),
                        (ns+'/target_pose', target), (ns+'/target_valid', Record(data=True)),
                        (ns+'/move_base/local_costmap/costmap', grid), ('/odom', odom)]
            records.extend((name, msg, Stamp(t)) for name, msg in messages)

        class Bag(object):
            def __init__(self, path):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

            def read_messages(self, topics):
                return (r for r in records if r[0] in topics)

        module.Bag = Bag
        previous = sys.modules.get('rosbag')
        sys.modules['rosbag'] = module
        try:
            rows = acceptance.bag_samples('ignored.bag', self.trial)
            result = acceptance.summarize(rows, self.trial)
            self.assertEqual(result['status'], 'PASS', result)
            for topic, message, stamp in records:
                if topic.endswith('/perception_status'):
                    message.data = json.dumps(dict(rgb_stamp=100., depth_stamp=100.))
            stale = acceptance.summarize(acceptance.bag_samples('ignored.bag', self.trial), self.trial)
            self.assertEqual(stale['status'], 'INCOMPLETE')
            self.assertIn('insufficient_fresh_rgb', stale['missing'])
            self.assertIn('insufficient_fresh_depth', stale['missing'])
        finally:
            if previous is None:
                del sys.modules['rosbag']
            else:
                sys.modules['rosbag'] = previous


if __name__ == '__main__':
    unittest.main()
