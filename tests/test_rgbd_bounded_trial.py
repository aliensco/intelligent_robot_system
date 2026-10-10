"""Offline fault injection for the bounded commissioning relay, no ROS/hardware."""
import os
import sys
import unittest

PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'tools', 'run_rgbd_bounded_trial.py')
if sys.version_info[0] < 3:
    import imp
    MODULE = imp.load_source('bounded_trial', PATH)
else:
    import importlib.util
    SPEC = importlib.util.spec_from_file_location('bounded_trial', PATH)
    MODULE = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(MODULE)


class BoundedTrialTests(unittest.TestCase):
    def ready(self, trial=None, now=10., command=(.04, 0., 0.)):
        trial = trial or MODULE.Trial(now)
        for key, value in [('command', command), ('status', 'shadow_velocity_only'),
                           ('target', True), ('estop', False), ('lease', True),
                           ('sensors', True), ('graph', True)]:
            trial.update(key, value, now)
        trial.odom(0., 0., 0., 0., 0., 0., now, now, now)
        return trial

    def test_waits_without_all_inputs_and_has_bounded_preflight(self):
        trial = MODULE.Trial(10.)
        self.assertEqual(trial.output(10.), (0., 0.))
        self.assertIsNone(trial.started)
        self.assertEqual(trial.output(25.), (0., 0.))
        self.assertTrue(trial.terminal)

    def test_relays_exact_curvature_and_rejects_oversized_not_clips(self):
        trial = self.ready(command=(.04, .08, 0.))
        self.assertEqual(trial.output(10.), (.04, .08))
        trial.update('command', (.051, .08, 0.), 10.01)
        self.assertEqual(trial.output(10.01), (0., 0.))
        trial.update('command', (.04, .08, 0.), 10.02)
        self.assertEqual(trial.output(10.02), (0., 0.))

    def test_all_stale_inputs_latch_stop_after_armed(self):
        for key in ('command', 'status', 'target', 'estop', 'lease', 'sensors', 'odom', 'graph'):
            trial = self.ready()
            trial.output(10.)
            value, unused = trial.data[key]
            trial.update(key, value, 9.)
            self.assertEqual(trial.output(10.01), (0., 0.), key)
            self.assertTrue(trial.terminal, key)
            trial.update(key, value, 10.02)
            self.assertEqual(trial.output(10.02), (0., 0.), key)

    def test_estop_collision_lease_target_and_gate_rejection(self):
        for key, value in [('estop', True), ('sensors', False), ('lease', False),
                           ('graph', False), ('target', False), ('status', 'swept_footprint_blocked')]:
            trial = self.ready()
            trial.output(10.)
            trial.update(key, value, 10.01)
            self.assertEqual(trial.output(10.01), (0., 0.))
            self.assertTrue(trial.terminal)

    def test_transient_emergency_callbacks_cannot_clear_before_output_tick(self):
        for key, bad, good in [('estop', True, False), ('sensors', False, True),
                               ('lease', False, True), ('target', False, True)]:
            trial = self.ready()
            trial.output(10.)
            trial.update(key, bad, 10.001)
            trial.update(key, good, 10.002)
            self.assertEqual(trial.output(10.003), (0., 0.))
            self.assertTrue(trial.terminal)

    def test_sensor_stamp_age_is_retained_in_wall_freshness_check(self):
        trial = self.ready()
        trial.output(10.)
        trial.odom(.001, 0., 0., .04, 0., 0., 10.01, 10.3, 10.1)
        self.assertEqual(trial.output(10.12), (0., 0.))
        self.assertEqual(trial.reason, 'stale_odom')

    def test_path_length_counts_forward_then_reverse_not_net_displacement(self):
        trial = self.ready()
        trial.output(10.)
        for i, x in enumerate((.02, .04, .02, .0)):
            now = 10. + (i + 1) * .04
            trial.odom(x, 0., 0., .04, 0., 0., now, now, now)
        self.assertAlmostEqual(trial.distance, .08)
        self.assertEqual(trial.output(10.16), (0., 0.))
        self.assertEqual(trial.reason, 'travel_budget')

    def test_predictive_stopping_reserve_precedes_ten_centimeters(self):
        trial = self.ready(command=(.05, 0., 0.))
        trial.output(10.)
        for i in range(1, 5):
            now = 10. + i * .02
            trial.odom(i*.018, 0., 0., .05, 0., 0., now, now, now)
        self.assertEqual(trial.output(10.08), (0., 0.))
        self.assertLess(trial.distance + .05*.3 + .05**2/.6, .1)

    def test_rotation_budget_counts_absolute_turn_and_reserves_braking(self):
        trial = self.ready(command=(0., .15, 0.))
        trial.output(10.)
        trial.odom(0., 0., .03, 0., 0., .15, 10.02, 10.02, 10.02)
        trial.odom(0., 0., 0., 0., 0., .15, 10.04, 10.04, 10.04)
        self.assertAlmostEqual(trial.yaw, .06)
        self.assertEqual(trial.output(10.04), (0., 0.))
        self.assertEqual(trial.reason, 'rotation_budget')

    def test_old_nan_reordered_or_jumping_odometry_stops(self):
        for x, stamp, ros_now in [(float('nan'), 10.1, 10.1), (0., 10., 10.1),
                                  (0., 10.1, 10.5), (.5, 10.1, 10.1)]:
            trial = self.ready()
            trial.output(10.)
            trial.odom(x, 0., 0., 0., 0., 0., stamp, ros_now, 10.1)
            self.assertEqual(trial.output(10.1), (0., 0.))
            self.assertTrue(trial.terminal)

    def test_duration_stops_even_if_all_messages_keep_arriving(self):
        trial = self.ready()
        trial.output(10.)
        self.ready(trial=trial, now=14.01)
        self.assertEqual(trial.output(14.01), (0., 0.))
        self.assertEqual(trial.reason, 'duration_limit')

    def test_overspeed_or_unsupported_axes_stop(self):
        trial = self.ready()
        trial.output(10.)
        trial.odom(.001, 0., 0., .07, 0., 0., 10.01, 10.01, 10.01)
        self.assertEqual(trial.output(10.01), (0., 0.))
        self.assertEqual(trial.reason, 'measured_speed_exceeded')
        trial = self.ready(command=(.04, 0., .01))
        self.assertEqual(trial.output(10.), (0., 0.))

    def test_no_motion_when_approved_gate_is_still_zero(self):
        trial = self.ready(command=(0., 0., 0.))
        self.assertEqual(trial.output(10.), (0., 0.))
        self.assertIsNone(trial.started)

    def test_limits_cannot_be_raised_beyond_authorized_trial(self):
        for keyword in ({'max_linear': .051}, {'max_angular': .16}, {'distance': .081},
                        {'yaw_limit': .11}, {'duration': 9.}, {'duration': float('nan')}):
            with self.assertRaises(ValueError):
                MODULE.Trial(10., **keyword)

    def test_rejected_request_is_distinct_from_actual_output_peak(self):
        trial = self.ready(command=(.06, -.08, 0.))
        v, w = trial.output(10.)
        trial.diagnostics.velocity('output', v, w)
        result = trial.diagnostics.summary()
        self.assertEqual(result['peak_requested_linear_m_s'], .06)
        self.assertEqual(result['peak_requested_angular_rad_s'], .08)
        self.assertEqual(result['peak_output_linear_m_s'], 0.)
        self.assertEqual(result['peak_output_angular_rad_s'], 0.)
        self.assertEqual(trial.reason, 'command_out_of_bounds')

    def test_turn_request_with_pwm_but_no_encoder_or_odom_motion_is_visible(self):
        trial = self.ready(command=(0., -.05, 0.))
        trial.diagnostics.sensors(123, 456, 0, 0)
        v, w = trial.output(10.)
        trial.diagnostics.velocity('output', v, w)
        trial.diagnostics.sensors(123, 456, -3, 1)
        trial.odom(0., 0., 0., 0., 0., 0., 10.1, 10.1, 10.1)
        trial.stop('duration_limit')
        trial.diagnostics.sensors(123, 456, 0, 0)
        result = trial.diagnostics.summary()
        self.assertEqual(result['peak_requested_angular_rad_s'], .05)
        self.assertEqual(result['peak_output_angular_rad_s'], .05)
        self.assertEqual(result['peak_measured_linear_m_s'], 0.)
        self.assertEqual(result['peak_measured_angular_rad_s'], 0.)
        self.assertEqual(result['start_encoders_left_right'], [123, 456])
        self.assertEqual(result['end_encoders_left_right'], [123, 456])
        self.assertEqual(result['encoder_delta_left_right'], [0, 0])
        self.assertEqual(result['peak_abs_pwm_left_right'], [3, 1])
        self.assertEqual(result['peak_abs_pwm'], 3)
        self.assertEqual(result['sensor_samples'], 3)

    def test_diagnostics_preserve_encoder_wrap_and_braking_tail_motion(self):
        trial = self.ready()
        trial.diagnostics.sensors(65534, 2, -1, 1)
        trial.output(10.)
        trial.stop('duration_limit')
        trial.odom(.001, 0., 0., .03, .004, -.02, 10.1, 10.1, 10.1)
        trial.diagnostics.sensors(2, 65534, 0, -4)
        result = trial.diagnostics.summary()
        self.assertAlmostEqual(result['peak_measured_linear_m_s'], (.03**2+.004**2)**.5)
        self.assertEqual(result['peak_measured_angular_rad_s'], .02)
        self.assertEqual(result['encoder_delta_left_right'], [4, -4])
        self.assertEqual(result['peak_abs_pwm_left_right'], [1, 4])
        self.assertEqual(trial.reason, 'duration_limit')

    def test_missing_diagnostics_are_null_not_false_zero_evidence(self):
        stats = MODULE.Diagnostics()
        stats.velocity('measured', float('nan'), float('inf'))
        stats.sensors(None, 0, 0, 0)
        result = stats.summary()
        self.assertIsNone(result['peak_measured_linear_m_s'])
        self.assertIsNone(result['peak_measured_angular_rad_s'])
        self.assertIsNone(result['encoder_delta_left_right'])
        self.assertIsNone(result['peak_abs_pwm'])
        self.assertEqual(result['sensor_samples'], 0)


if __name__ == '__main__':
    unittest.main()
