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
    def field_speed_trial(self):
        trial = self.ready(MODULE.Trial(10., field_test=True, max_linear=.10,
                                      max_angular=.30, distance=1., yaw_limit=2.,
                                      resume_after_target_loss=True), command=(.08, .30, 0.))
        self.assertEqual(trial.output(10.), (.08, .30))
        return trial

    def field_sample(self, trial, now, angular, linear=0., lateral=0., target=True):
        for key, value in [('command', (.08, .30, 0.)), ('status', 'shadow_velocity_only'),
                           ('target', target), ('estop', False), ('lease', True),
                           ('sensors', True), ('graph', True)]:
            trial.update(key, value, now)
        trial.odom(0., 0., 0., linear, lateral, angular, now, now, now)

    def test_field_angular_single_spike_pauses_and_recovers_on_new_source_stamps(self):
        trial = self.field_speed_trial()
        trial.distance, trial.yaw = .2, .3
        self.field_sample(trial, 10.02, .33946654)
        self.assertEqual(trial.output(10.02), (0., 0.))
        self.assertEqual(trial.reason, 'paused_for_angular_overspeed')
        self.assertFalse(trial.terminal)
        self.field_sample(trial, 10.04, .295)
        self.assertEqual(trial.output(10.04), (0., 0.))
        self.assertEqual(trial.reason, 'paused_for_angular_recovery')
        # Loop time alone cannot complete a source-timestamp recovery interval.
        self.assertEqual(trial.output(10.13), (0., 0.))
        self.field_sample(trial, 10.13, .295)
        self.assertEqual(trial.output(10.13), (0., 0.))
        self.field_sample(trial, 10.14, .295)
        self.assertEqual(trial.output(10.14), (.08, .30))
        self.assertEqual(trial.angular_pause_count, 1)
        self.assertEqual(trial.started, 10.)
        self.assertEqual((trial.distance, trial.yaw), (.2, .3))
        self.assertEqual(trial.max_angular, .30)
        self.assertAlmostEqual(trial.angular_soft_limit, .33)
        self.assertAlmostEqual(trial.angular_hard_limit, .40)

    def test_field_sustained_angular_overspeed_stops_during_target_pause(self):
        trial = self.field_speed_trial()
        for now in (10.02, 10.07):
            self.field_sample(trial, now, -.34, target=False)
            trial.update('status', 'swept_footprint_blocked', now)
            self.assertEqual(trial.output(now), (0., 0.))
            self.assertFalse(trial.terminal)
        self.field_sample(trial, 10.12, -.34, target=False)
        self.assertEqual(trial.output(10.12), (0., 0.))
        self.assertTrue(trial.terminal)
        self.assertEqual(trial.reason, 'measured_angular_speed_sustained')

    def test_field_severe_angular_overspeed_is_immediately_terminal(self):
        trial = self.field_speed_trial()
        self.field_sample(trial, 10.02, -.401, target=False)
        self.assertTrue(trial.terminal)
        self.assertEqual(trial.output(10.02), (0., 0.))
        self.assertEqual(trial.reason, 'measured_angular_speed_hard_limit')

    def test_field_recovery_requires_below_command_cap_and_resets_after_excursion(self):
        trial = self.field_speed_trial()
        self.field_sample(trial, 10.02, .339)
        self.field_sample(trial, 10.04, .29)
        self.field_sample(trial, 10.10, .31)
        self.assertIsNone(trial.angular_recovery_since)
        self.field_sample(trial, 10.12, .29)
        self.field_sample(trial, 10.20, .29)
        self.assertEqual(trial.output(10.20), (0., 0.))
        self.field_sample(trial, 10.22, .29)
        self.assertEqual(trial.output(10.22), (.08, .30))

    def test_field_spike_is_latched_even_between_output_ticks(self):
        trial = self.field_speed_trial()
        self.field_sample(trial, 10.02, .339)
        self.field_sample(trial, 10.04, .29)
        self.assertEqual(trial.output(10.041), (0., 0.))
        self.assertTrue(trial.angular_paused)

    def test_field_angular_pause_cannot_hide_stale_or_reordered_odometry(self):
        trial = self.field_speed_trial()
        self.field_sample(trial, 10.02, .339)
        self.field_sample(trial, 10.04, .29)
        trial.odom(0., 0., 0., 0., 0., .29, 10.04, 10.14, 10.14)
        self.assertEqual(trial.reason, 'invalid_or_reordered_odometry')
        self.assertEqual(trial.output(10.14), (0., 0.))
        trial = self.field_speed_trial()
        self.field_sample(trial, 10.02, .339)
        for name, value in list(trial.data.items()):
            if name != 'odom':
                trial.update(name, value[0], 10.34)
        self.assertEqual(trial.output(10.34), (0., 0.))
        self.assertEqual(trial.reason, 'stale_odom')
        self.assertTrue(trial.terminal)

    def test_field_linear_lateral_and_duration_guards_remain_immediate_during_pause(self):
        for linear, lateral in ((.12, 0.), (0., .02)):
            trial = self.field_speed_trial()
            self.field_sample(trial, 10.02, .339)
            self.field_sample(trial, 10.04, .29, linear, lateral, target=False)
            self.assertEqual(trial.reason, 'measured_speed_exceeded')
            self.assertTrue(trial.terminal)
        trial = self.field_speed_trial()
        self.field_sample(trial, 10.02, .339)
        self.assertEqual(trial.output(14.01), (0., 0.))
        self.assertEqual(trial.reason, 'duration_limit')

    def test_strict_commissioning_keeps_single_sample_angular_stop(self):
        trial = self.ready(command=(.04, .15, 0.))
        trial.output(10.)
        trial.odom(0., 0., 0., 0., 0., .181, 10.02, 10.02, 10.02)
        self.assertEqual(trial.output(10.02), (0., 0.))
        self.assertEqual(trial.reason, 'measured_speed_exceeded')
        self.assertTrue(trial.terminal)

    def test_field_odometry_allows_legal_turn_over_longer_fresh_interval(self):
        trial = self.ready(MODULE.Trial(10., field_test=True, max_linear=.10,
                                      max_angular=.30, distance=1., yaw_limit=2.))
        trial.output(10.)
        trial.odom(.018, 0., .054, .10, 0., .30, 10.18, 10.18, 10.18)
        self.assertFalse(trial.terminal)
        trial.odom(.05, 0., .20, .10, 0., .30, 10.20, 10.20, 10.20)
        self.assertEqual(trial.reason, 'odometry_jump')

    def test_encoders_accumulate_beyond_half_counter_range(self):
        d = MODULE.Diagnostics()
        for value in range(0, 100001, 1000):
            d.sensors(value % 65536, (-value) % 65536, 10, 10)
        self.assertEqual(d.summary()['encoder_delta_left_right'], [100000, -100000])

    def test_field_speed_profile_is_explicit_and_measured_speed_remains_checked(self):
        with self.assertRaises(ValueError):
            MODULE.Trial(10., max_linear=.10, max_angular=.30)
        trial = MODULE.Trial(10., field_test=True, max_linear=.10, max_angular=.30,
                             distance=1., yaw_limit=2.)
        self.ready(trial, command=(.10, .25, 0.))
        self.assertEqual(trial.output(10.), (.10, .25))
        self.ready(trial, now=10.02, command=(.10, .25, 0.))
        trial.data['odom'] = ((.12, 0., 0.), 10.02)
        self.assertEqual(trial.output(10.02), (0., 0.))
        self.assertEqual(trial.reason, 'measured_speed_exceeded')

    def test_field_missing_target_messages_pause_then_resume_on_fresh_data(self):
        trial = self.ready(MODULE.Trial(10., duration=60., distance=1., yaw_limit=1.,
                                      resume_after_target_loss=True, field_test=True))
        trial.output(10.)
        self.ready(trial, now=11.)
        trial.data['target'] = (True, 10.)
        self.assertEqual(trial.output(11.), (0., 0.))
        self.assertFalse(trial.terminal)
        self.assertEqual(trial.reason, 'paused_for_stale_target')
        self.ready(trial, now=11.1)
        self.assertEqual(trial.output(11.1), (.04, 0.))
        self.assertEqual(trial.started, 10.)

    def test_field_perception_pause_cannot_hide_stale_base_sensors(self):
        trial = self.ready(MODULE.Trial(10., duration=60., distance=1., yaw_limit=1.,
                                      resume_after_target_loss=True, field_test=True))
        trial.output(10.)
        self.ready(trial, now=11.)
        trial.data['target'] = (True, 10.)
        trial.data['sensors'] = (True, 10.)
        self.assertEqual(trial.output(11.), (0., 0.))
        self.assertTrue(trial.terminal)
        self.assertEqual(trial.reason, 'stale_sensors')

    def test_long_session_requires_explicit_field_mode(self):
        with self.assertRaises(ValueError):
            MODULE.Trial(10., duration=60., distance=1.)
        trial = self.ready(MODULE.Trial(10., duration=60., distance=1., yaw_limit=1.,
                                      resume_after_target_loss=True, field_test=True))
        self.assertEqual(trial.output(10.), (.04, 0.))
        trial.distance = .20
        self.ready(trial, now=11.)
        self.assertEqual(trial.output(11.), (.04, 0.))
        trial.distance = 1.
        self.assertEqual(trial.output(11.01), (0., 0.))
        self.assertEqual(trial.reason, 'travel_budget')

    def test_field_hold_keeps_zero_and_can_resume_without_renewing_budget(self):
        trial = self.ready(MODULE.Trial(10., duration=60., distance=1., yaw_limit=1.,
                                      resume_after_target_loss=True, field_test=True))
        trial.output(10.)
        self.ready(trial, now=11.)
        trial.update('status', 'permission_or_command_stale', 11.)
        self.assertEqual(trial.output(11.), (0., 0.))
        self.ready(trial, now=20.)
        trial.update('status', 'permission_or_command_stale', 20.)
        self.assertEqual(trial.output(20.), (0., 0.))
        self.assertFalse(trial.terminal)
        self.ready(trial, now=21.)
        self.assertEqual(trial.output(21.), (.04, 0.))
        self.assertEqual(trial.started, 10.)
        trial.update('sensors', False, 21.01)
        self.assertEqual(trial.output(21.01), (0., 0.))
        self.assertTrue(trial.terminal)

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

    def test_optional_recovery_pauses_zero_without_resetting_trial_budget(self):
        trial = self.ready(MODULE.Trial(10., resume_after_target_loss=True))
        self.assertEqual(trial.output(10.), (.04, 0.))
        trial.update('target', False, 10.02)
        self.assertEqual(trial.output(10.02), (0., 0.))
        self.assertFalse(trial.terminal)
        self.assertEqual(trial.started, 10.)
        self.ready(trial, now=10.10)
        self.assertEqual(trial.output(10.10), (.04, 0.))
        self.assertEqual(trial.started, 10.)
        self.ready(trial, now=14.01)
        self.assertEqual(trial.output(14.01), (0., 0.))
        self.assertEqual(trial.reason, 'duration_limit')

    def test_optional_recovery_does_not_resume_after_emergency(self):
        trial = self.ready(MODULE.Trial(10., resume_after_target_loss=True))
        trial.output(10.)
        trial.update('sensors', False, 10.02)
        self.ready(trial, now=10.1)
        self.assertEqual(trial.output(10.1), (0., 0.))
        self.assertTrue(trial.terminal)

    def test_relays_exact_curvature_and_rejects_oversized_not_clips(self):
        trial = self.ready(command=(.04, .08, 0.))
        self.assertEqual(trial.output(10.), (.04, .08))
        trial.update('command', (.051, .08, 0.), 10.01)
        self.assertEqual(trial.output(10.01), (0., 0.))
        trial.update('command', (.04, .08, 0.), 10.02)
        self.assertEqual(trial.output(10.02), (0., 0.))

    def test_float32_limits_are_accepted_but_output_is_clamped(self):
        import struct
        f32 = lambda v: struct.unpack('f', struct.pack('f', v))[0]
        for sign in (-1, 1):
            trial = self.ready(command=(f32(.05), sign*f32(.15), 0.))
            self.assertEqual(trial.output(10.), (.05, sign*.15))
            self.assertFalse(trial.terminal)
        trial = self.ready(command=(.05, .15001, 0.))
        self.assertEqual(trial.output(10.), (0., 0.))
        self.assertEqual(trial.reason, 'command_out_of_bounds')

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
