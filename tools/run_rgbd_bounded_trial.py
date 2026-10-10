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
        self.end_encoders = encoders
        self.peak_pwm = [max(a, b) for a, b in zip(self.peak_pwm or [0, 0], pwm)]
        self.sensor_samples += 1

    def summary(self):
        result = dict(self.peaks)
        # Kobuki has unsigned 16-bit encoders; this tiny trial cannot turn a
        # wheel by half the counter range. Preserve raw readings as evidence.
        delta = None if self.start_encoders is None else [
            (end - start + 32768) % 65536 - 32768
            for start, end in zip(self.start_encoders, self.end_encoders)]
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
    """Pure state machine. A stop after arming is terminal, never auto-resumes."""
    def __init__(self, created, duration=4., distance=.06, yaw_limit=.1,
                 max_linear=.05, max_angular=.15, preflight=15.):
        values = (created, duration, distance, yaw_limit, max_linear, max_angular, preflight)
        if not all(finite(v) for v in values) or not (
                0 < duration <= 8 and 0 < distance <= .08 and 0 < yaw_limit <= .1 and
                0 < max_linear <= .05 and 0 < max_angular <= .15 and 0 < preflight <= 30):
            raise ValueError('trial bounds exceed commissioning limits')
        self.created, self.duration, self.limit = created, duration, distance
        self.yaw_limit, self.max_linear, self.max_angular = yaw_limit, max_linear, max_angular
        self.preflight, self.started = preflight, None
        self.data, self.previous = {}, None
        self.distance = self.yaw = 0.
        self.reason, self.terminal = 'waiting_for_inputs', False
        self.diagnostics = Diagnostics()

    def update(self, name, value, now):
        self.data[name] = (value, now)
        if name == 'command':
            self.diagnostics.velocity('requested', value[0], value[1])
        # Latch transient emergency events even if the next callback clears them
        # before the 50 Hz output loop runs.
        if ((name == 'estop' and value) or
                (name in ('sensors', 'lease') and not value)):
            self.stop('event_veto:' + name)
        if self.started is not None and ((name in ('target', 'graph') and not value) or
                                        (name == 'status' and value not in APPROVED)):
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
            px, py, pyaw, unused = self.previous
            step = math.hypot(x - px, y - py)
            turn = abs(math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw)))
            self.distance += step
            self.yaw += turn
            if step > .03 or turn > .05:
                self.stop('odometry_jump')
        self.previous = (x, y, yaw, stamp)
        self.update('odom', (v, lateral, w), now - (ros_now - stamp))

    def output(self, now):
        if self.terminal:
            return (0., 0.)
        if self.started is None and now - self.created >= self.preflight:
            return self.stop('preflight_timeout:' + self.reason)
        if self.started is not None and now - self.started >= self.duration:
            return self.stop('duration_limit')
        for name in ('command', 'status', 'target', 'estop', 'lease', 'sensors', 'odom', 'graph'):
            age = .5 if name == 'graph' else (.3 if name == 'odom' else .2)
            if name not in self.data or not 0 <= now - self.data[name][1] <= age:
                self.reason = 'stale_' + name
                return self.stop(self.reason) if self.started is not None else (0., 0.)
        d = dict((key, value[0]) for key, value in self.data.items())
        if d['estop'] or not d['lease'] or not d['sensors']:
            return self.stop('estop_lease_sensor_or_graph_veto')
        if not d['graph']:
            self.reason = 'waiting_for_exclusive_topic_graph'
            return self.stop(self.reason) if self.started is not None else (0., 0.)
        if not d['target'] or d['status'] not in APPROVED:
            self.reason = 'target_or_gate_veto:' + str(d['status'])
            return self.stop(self.reason) if self.started is not None else (0., 0.)
        v, w, unsupported = d['command']
        if (not all(finite(q) for q in (v, w, unsupported)) or unsupported > 1e-6 or
                not 0 <= v <= self.max_linear + 1e-9 or abs(w) > self.max_angular + 1e-9):
            return self.stop('command_out_of_bounds')
        measured, lateral, angular = d['odom']
        if abs(measured) > .065 or abs(lateral) > .015 or abs(angular) > .18:
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
        if self.distance >= self.limit or self.distance + speed * .3 + speed ** 2 / .6 >= .09:
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
                  args.max_linear, args.max_angular, args.preflight)
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
                    print(json.dumps({'reason': trial.reason, 'distance_m': trial.distance,
                                      'yaw_rad': trial.yaw, 'armed': trial.started is not None}))
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
        print(json.dumps(record))
        rospy.signal_shutdown('bounded trial complete')


if __name__ == '__main__':
    main()
