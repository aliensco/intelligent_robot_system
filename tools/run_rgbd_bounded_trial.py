#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""One bounded commissioning trial using genuine method-3 shadow gate outputs.

Python 2.7/3 compatible. Does not set calibration flags or create navigation
commands. Requires a running, verified base command timeout: a killed process,
blocked ROS transport or suspended VM cannot be stopped by a Python watchdog.
The default 0.06 m distance is an early stop trigger, not guaranteed final travel.
The stopping reserve uses assumed braking; actual travel is recorded through the
0.5 s stopped tail. Diagnostics include preflight and that stopped tail.
"""
from __future__ import print_function
import argparse
import json
import math
import signal
import threading
import time


APPROVED = frozenset(('shadow_velocity_only', 'shadow_visibility_recenter',
                      'visibility_or_clearance_slowdown'))


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


class Diagnostics(object):
    """Passive evidence; missing data is null, never reported as measured zero."""
    def __init__(self):
        self.peaks = {}
        for source in ('requested', 'output', 'measured'):
            for axis in ('linear_m_s', 'angular_rad_s'):
                self.peaks['peak_' + source + '_' + axis] = None
        self.start_encoders = self.end_encoders = None
        self.encoder_totals = [0, 0]
        self.peak_pwm = None
        self.sensor_samples = 0

    def velocity(self, source, v, w):
        for axis, value in (('linear_m_s', v), ('angular_rad_s', w)):
            if finite(value):
                key = 'peak_' + source + '_' + axis
                self.peaks[key] = max(self.peaks[key] or 0., abs(value))

    def sensors(self, left_encoder, right_encoder, left_pwm, right_pwm):
        values = (left_encoder, right_encoder, left_pwm, right_pwm)
        if any(value is None or not finite(value) for value in values):
            return
        encoders, pwm = [int(q) for q in values[:2]], [abs(int(q)) for q in values[2:]]
        if self.start_encoders is None:
            self.start_encoders = encoders[:]
        else:
            self.encoder_totals = [total+(end-start+32768) % 65536-32768
                                   for total, start, end in zip(self.encoder_totals, self.end_encoders, encoders)]
        self.end_encoders = encoders
        self.peak_pwm = [max(a, b) for a, b in zip(self.peak_pwm or [0, 0], pwm)]
        self.sensor_samples += 1

    def summary(self):
        result = dict(self.peaks)
        # Unwrap every sensor sample: a full field run can exceed 32768 ticks.
        delta = None if self.start_encoders is None else self.encoder_totals[:]
        result.update({'start_encoders_left_right': self.start_encoders,
                       'end_encoders_left_right': self.end_encoders,
                       'encoder_delta_left_right': delta,
                       'peak_abs_pwm_left_right': self.peak_pwm,
                       'peak_abs_pwm': max(self.peak_pwm) if self.peak_pwm is not None else None,
                       'sensor_samples': self.sensor_samples})
        return result


def monotonic_clock():
    if hasattr(time, 'monotonic'):
        return time.monotonic
    # ROS Kinetic's Python 2 lacks time.monotonic. Never use adjustable wall time.
    import ctypes
    class Timespec(ctypes.Structure):
        _fields_ = [('seconds', ctypes.c_long), ('nanoseconds', ctypes.c_long)]
    clock_gettime = ctypes.CDLL('librt.so.1', use_errno=True).clock_gettime
    def read():
        result = Timespec()
        if clock_gettime(1, ctypes.byref(result)) != 0:
            raise RuntimeError('CLOCK_MONOTONIC unavailable')
        return result.seconds + result.nanoseconds * 1e-9
    return read


class Trial(object):
    """Bounded relay; field mode can hold at zero while perception recovers."""
    def __init__(self, created, duration=4., distance=.06, yaw_limit=.1,
                 max_linear=.05, max_angular=.15, preflight=15., resume_after_target_loss=False,
                 field_test=False):
        values = (created, duration, distance, yaw_limit, max_linear, max_angular, preflight)
        if not all(finite(v) for v in values) or not (
                0 < duration <= (300 if field_test else 8) and
                0 < distance <= (5 if field_test else .08) and
                0 < yaw_limit <= (2*math.pi if field_test else .1) and
                0 < max_linear <= (.10 if field_test else .05) and
                0 < max_angular <= (.30 if field_test else .15) and 0 < preflight <= 30):
            raise ValueError('trial bounds exceed commissioning limits')
        self.created, self.duration, self.limit = created, duration, distance
        self.yaw_limit, self.max_linear, self.max_angular = yaw_limit, max_linear, max_angular
        self.preflight, self.started = preflight, None
        self.resume_after_target_loss = resume_after_target_loss
        self.field_test = field_test
        self.paused_since = None
        self.data, self.previous = {}, None
        self.distance = self.yaw = 0.
        self.reason, self.terminal = 'waiting_for_inputs', False
        self.diagnostics = Diagnostics()
        self.angular_soft_limit = max_angular+.03
        self.angular_hard_limit = max_angular+max(.10, max_angular*.30)
        self.angular_paused = False
        self.angular_over_since = self.angular_recovery_since = None
        self.angular_pause_reason = None
        self.angular_pause_count = 0

    def field_measured_speed(self, v, lateral, w, stamp):
        """Only new, validated odometry can advance field overspeed recovery.

        The hard threshold ends the session; it is NOT a permitted drive rate.
        Every sample above the unchanged soft limit latches zero output until
        measured rotation remains within the command cap for 0.10 source seconds.
        """
        if abs(v) > self.max_linear+.015 or abs(lateral) > .015:
            self.stop('measured_speed_exceeded')
            return
        rotation = abs(w)
        if rotation > self.angular_hard_limit:
            self.stop('measured_angular_speed_hard_limit')
            return
        if rotation > self.angular_soft_limit:
            if not self.angular_paused:
                self.angular_pause_count += 1
            self.angular_paused = True
            self.angular_pause_reason = 'paused_for_angular_overspeed'
            self.angular_recovery_since = None
            if self.angular_over_since is None:
                self.angular_over_since = stamp
            if stamp-self.angular_over_since >= .10-1e-6:
                self.stop('measured_angular_speed_sustained')
        elif self.angular_paused:
            self.angular_over_since = None
            self.angular_pause_reason = 'paused_for_angular_recovery'
            if rotation <= self.max_angular:
                if self.angular_recovery_since is None:
                    self.angular_recovery_since = stamp
                if stamp-self.angular_recovery_since >= .10-1e-6:
                    self.angular_paused = False
                    self.angular_recovery_since = None
                    self.angular_pause_reason = None
            else:
                # Hysteresis: falling just below the soft limit is insufficient.
                self.angular_recovery_since = None

    def update(self, name, value, now):
        self.data[name] = (value, now)
        if name == 'command':
            self.diagnostics.velocity('requested', value[0], value[1])
        # Latch transient emergency events even if the next callback clears them
        # before the 50 Hz output loop runs.
        if ((name == 'estop' and value) or
                (name in ('sensors', 'lease') and not value)):
            self.stop('event_veto:' + name)
        if self.started is not None and ((name == 'graph' and not value) or
                (not self.resume_after_target_loss and
                 ((name == 'target' and not value) or (name == 'status' and value not in APPROVED)))):
            self.stop('event_veto:' + name)

    def stop(self, reason):
        if not self.terminal:
            self.reason, self.terminal = reason, True
        return (0., 0.)

    def odom(self, x, y, yaw, v, lateral, w, stamp, ros_now, now):
        values = (x, y, yaw, v, lateral, w, stamp, ros_now)
        if (not all(finite(q) for q in values) or stamp <= 0 or
                not 0 <= ros_now - stamp <= .3 or
                (self.previous is not None and stamp <= self.previous[3])):
            self.data.pop('odom', None)
            if self.started is not None:
                self.stop('invalid_or_reordered_odometry')
            return
        self.diagnostics.velocity('measured', math.hypot(v, lateral), w)
        if self.previous is not None and self.started is not None:
            px, py, pyaw, previous_stamp = self.previous
            step = math.hypot(x - px, y - py)
            turn = abs(math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw)))
            self.distance += step
            self.yaw += turn
            elapsed = stamp-previous_stamp
            if (step > max(.03, (self.max_linear+.015)*elapsed+.005) or
                    turn > max(.05, (self.max_angular+.03)*elapsed+.005)):
                self.stop('odometry_jump')
        self.previous = (x, y, yaw, stamp)
        self.update('odom', (v, lateral, w), now - (ros_now - stamp))
        if self.field_test and self.started is not None and not self.terminal:
            # Observe here, not after target/gate checks: a perception pause
            # cannot hide persistent physical overspeed or a one-frame spike.
            self.field_measured_speed(v, lateral, w, stamp)

    def output(self, now):
        if self.terminal:
            return (0., 0.)
        if self.started is None and now - self.created >= self.preflight:
            return self.stop('preflight_timeout:' + self.reason)
        if self.started is not None and now - self.started >= self.duration:
            return self.stop('duration_limit')
        stale = []
        for name in ('command', 'status', 'target', 'estop', 'lease', 'sensors', 'odom', 'graph'):
            # Match the upstream 0.5 s RGB-D watchdog and 0.3 s gate command
            # timeout. An unrelated 0.2 s limit caused needless stop/restarts
            # when image processing briefly took more than one frame period.
            age = (.5 if name in ('graph', 'target') else
                   (.3 if name in ('odom', 'command', 'status') else .2))
            if name not in self.data or not 0 <= now - self.data[name][1] <= age:
                stale.append(name)
        critical = [name for name in stale if name not in ('command', 'status', 'target')]
        if self.field_test and self.started is not None and not critical:
            measured, lateral, unused_angular = self.data['odom'][0]
            if abs(measured) > self.max_linear+.015 or abs(lateral) > .015:
                return self.stop('measured_speed_exceeded')
            if self.angular_paused:
                self.reason = self.angular_pause_reason
                return (0., 0.)
        if stale:
            if self.started is not None and self.field_test and self.resume_after_target_loss and not critical:
                if self.paused_since is None:
                    self.paused_since = now
                if now-self.paused_since >= 60.:
                    return self.stop('target_or_gate_recovery_timeout')
                self.reason = 'paused_for_stale_' + stale[0]
                return (0., 0.)
            self.reason = 'stale_' + (critical[0] if critical else stale[0])
            return self.stop(self.reason) if self.started is not None else (0., 0.)
        d = dict((key, value[0]) for key, value in self.data.items())
        if d['estop'] or not d['lease'] or not d['sensors']:
            return self.stop('estop_lease_sensor_or_graph_veto')
        if not d['graph']:
            self.reason = 'waiting_for_exclusive_topic_graph'
            return self.stop(self.reason) if self.started is not None else (0., 0.)
        if not d['target'] or d['status'] not in APPROVED:
            self.reason = 'target_or_gate_veto:' + str(d['status'])
            if self.resume_after_target_loss and self.started is not None:
                if self.paused_since is None:
                    self.paused_since = now
                if now-self.paused_since >= (60. if self.field_test else 2.):
                    return self.stop('target_or_gate_recovery_timeout')
                # Emit zero on every cycle. Only fresh, fully confirmed target
                # AND approved gate output permit resumption; trial budgets
                # and odometry accumulation are never reset during a pause.
                self.reason = 'paused_for_target_or_gate'
                return (0., 0.)
            return self.stop(self.reason) if self.started is not None else (0., 0.)
        self.paused_since = None
        v, w, unsupported = d['command']
        if (not all(finite(q) for q in (v, w, unsupported)) or unsupported > 1e-6 or
                not 0 <= v <= self.max_linear + 1e-7 or abs(w) > self.max_angular + 1e-7):
            return self.stop('command_out_of_bounds')
        # DWA internally uses float32: 0.15 can arrive as 0.15000000596.
        # Accept only serialization-scale error, then clamp the REAL output.
        v = min(v, self.max_linear)
        w = max(-self.max_angular, min(w, self.max_angular))
        measured, lateral, angular = d['odom']
        if (abs(measured) > self.max_linear+.015 or abs(lateral) > .015 or
                abs(angular) > self.max_angular+.03):
            return self.stop('measured_speed_exceeded')
        if self.started is None:
            if abs(measured) > .01 or abs(angular) > .03:
                return self.stop('base_not_stationary_before_trial')
            if v == 0 and w == 0:
                self.reason = 'waiting_for_nonzero_gate_output'
                return (0., 0.)
            self.started = now
        speed, rotation = max(abs(measured), v), max(abs(angular), abs(w))
        # 0.3 s reaction and 0.3 m/s^2 braking, with 1 cm extra reserve.
        stopping_limit = self.limit + .03 if self.field_test else .09
        if self.distance >= self.limit or self.distance + speed * .3 + speed ** 2 / .6 >= stopping_limit:
            return self.stop('travel_budget')
        if self.yaw + rotation * .3 + rotation ** 2 / 1.2 >= self.yaw_limit:
            return self.stop('rotation_budget')
        self.reason = 'relaying_verified_shadow_output'
        return (v, w)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--namespace', default='/method3_motion')
    p.add_argument('--output', required=True, help='Explicit isolated physical velocity input')
    p.add_argument('--lease-topic', default='/method3_motion/test_enable')
    p.add_argument('--estop-topic', default='/method3_motion/test_estop')
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--sensors-topic', default='/mobile_base/sensors/core')
    p.add_argument('--duration', type=float, default=4.)
    p.add_argument('--distance', type=float, default=.06)
    p.add_argument('--yaw-limit', type=float, default=.1)
    p.add_argument('--max-linear', type=float, default=.05)
    p.add_argument('--max-angular', type=float, default=.15)
    p.add_argument('--preflight', type=float, default=15.)
    p.add_argument('--resume-after-target-loss', action='store_true',
                   help='Within the same bounded trial, pause at zero and resume after fresh target/gate confirmation')
    p.add_argument('--field-test', action='store_true',
                   help='Explicit field session: up to 300 s / 5 m / 0.10 m/s / 0.30 rad/s')
    p.add_argument('--odom-frame', default='odom')
    p.add_argument('--base-frame', default='base_footprint')
    return p


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from kobuki_msgs.msg import SensorState
    args = parser().parse_args(rospy.myargv()[1:])
    rospy.init_node('rgbd_bounded_trial', anonymous=True, disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise ValueError('live commissioning refuses simulated ROS time')
    prefix = args.namespace.rstrip('/')
    command_topic, status_topic, target_topic = [prefix + '/' + name for name in
                                               ('cmd_vel_preview', 'gate_status', 'target_valid')]
    topics = [command_topic, status_topic, target_topic, args.output, args.lease_topic,
              args.estop_topic, args.odom_topic, args.sensors_topic]
    if any(not topic.startswith('/') for topic in topics) or len(set(topics)) != len(topics):
        raise ValueError('all input/output topics must be distinct absolute names')
    clock = monotonic_clock()
    trial = Trial(clock(), args.duration, args.distance, args.yaw_limit,
                  args.max_linear, args.max_angular, args.preflight, args.resume_after_target_loss,
                  args.field_test)
    lock, killed = threading.RLock(), threading.Event()
    publisher = rospy.Publisher(args.output, Twist, queue_size=1)
    heartbeat = [clock()]
    interrupted = ['shutdown_or_interrupted']
    def zero():
        publisher.publish(Twist())
    def on_signal(unused_signum, unused_frame):
        interrupted[0] = 'operator_signal'
        killed.set()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)
    rospy.on_shutdown(killed.set)
    def observe(name, convert):
        def callback(msg):
            with lock:
                trial.update(name, convert(msg), clock())
        return callback
    def on_odom(msg):
        with lock:
            q, p, twist = msg.pose.pose.orientation, msg.pose.pose.position, msg.twist.twist
            norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if (msg.header.frame_id.lstrip('/') != args.odom_frame.lstrip('/') or
                    msg.child_frame_id.lstrip('/') != args.base_frame.lstrip('/') or
                    not finite(norm) or abs(norm - 1) > .02):
                trial.stop('invalid_odometry_frame_or_orientation')
                return
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            trial.odom(p.x, p.y, yaw, twist.linear.x, twist.linear.y, twist.angular.z,
                       msg.header.stamp.to_sec(), rospy.Time.now().to_sec(), clock())
    def on_sensors(msg):
        with lock:
            trial.update('sensors', not (msg.bumper or msg.cliff or msg.wheel_drop or msg.over_current), clock())
            trial.diagnostics.sensors(msg.left_encoder, msg.right_encoder, msg.left_pwm, msg.right_pwm)
    subscribers = [rospy.Subscriber(command_topic, Twist, observe('command', lambda m:
                    (m.linear.x, m.angular.z, abs(m.linear.y)+abs(m.linear.z)+abs(m.angular.x)+abs(m.angular.y))), queue_size=1),
        rospy.Subscriber(status_topic, String, observe('status', lambda m: m.data), queue_size=1),
        rospy.Subscriber(target_topic, Bool, observe('target', lambda m: m.data), queue_size=1),
        rospy.Subscriber(args.estop_topic, Bool, observe('estop', lambda m: m.data), queue_size=1),
        rospy.Subscriber(args.lease_topic, Bool, observe('lease', lambda m: m.data), queue_size=1),
        rospy.Subscriber(args.sensors_topic, SensorState, on_sensors, queue_size=1),
        rospy.Subscriber(args.odom_topic, Odometry, on_odom, queue_size=1)]
    def graph_monitor():
        while not killed.is_set() and not rospy.is_shutdown():
            try:
                code, unused, state = rospy.get_master().getSystemState()
                pubs, subs = dict(state[0]), dict(state[1])
                valid = (code == 1 and set(pubs.get(args.output, [])) == set([rospy.get_name()]) and
                         bool(subs.get(args.output)) and
                         set(pubs.get(command_topic, [])) == set([prefix + '/rgbd_velocity_gate']) and
                         set(pubs.get(status_topic, [])) == set([prefix + '/rgbd_velocity_gate']) and
                         set(pubs.get(target_topic, [])) == set([prefix + '/rgbd_target']))
            except Exception:
                valid = False
            with lock:
                trial.update('graph', valid, clock())
            killed.wait(.15)
    def watchdog():
        while not killed.wait(.02):
            if clock() - heartbeat[0] > .15:
                interrupted[0] = 'independent_wall_watchdog'
                killed.set()
                zero()
                return
    for target in (graph_monitor, watchdog):
        worker = threading.Thread(target=target)
        worker.daemon = True
        worker.start()
    last_reason = None
    try:
        while not killed.is_set() and not rospy.is_shutdown():
            with lock:
                v, w = trial.output(clock())
                out = Twist()
                if not killed.is_set():
                    out.linear.x, out.angular.z = v, w
                publisher.publish(out)
                trial.diagnostics.velocity('output', out.linear.x, out.angular.z)
                heartbeat[0] = clock()
                if trial.reason != last_reason:
                    status = {'reason': trial.reason, 'distance_m': trial.distance,
                              'yaw_rad': trial.yaw, 'armed': trial.started is not None}
                    if trial.field_test and trial.angular_paused:
                        status.update(angular_soft_limit_rad_s=trial.angular_soft_limit,
                                      angular_hard_limit_rad_s=trial.angular_hard_limit,
                                      measured_angular_rad_s=trial.data.get('odom', ((0., 0., None),))[0][2])
                    print(json.dumps(status))
                    last_reason = trial.reason
                if trial.terminal:
                    break
            killed.wait(.02)
    finally:
        killed.set()
        with lock:
            if not trial.terminal:
                trial.stop(interrupted[0])
        # Keep observing odometry during a short stopped tail for the final record.
        for unused in range(25):
            zero()
            time.sleep(.02)
        with lock:
            record = trial.diagnostics.summary()
            record.update({'final_reason': trial.reason, 'distance_m': trial.distance,
                           'yaw_rad': trial.yaw, 'armed': trial.started is not None,
                           'stop_trigger_distance_m': trial.limit,
                           'diagnostic_window': 'preflight_through_0.5s_stop_tail'})
            if trial.field_test:
                record.update(angular_overspeed_pauses=trial.angular_pause_count,
                              angular_soft_limit_rad_s=trial.angular_soft_limit,
                              angular_hard_limit_rad_s=trial.angular_hard_limit,
                              angular_recovery_hold_s=.10)
        print(json.dumps(record))
        rospy.signal_shutdown('bounded trial complete')


if __name__ == '__main__':
    main()
