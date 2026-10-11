#!/usr/bin/env python
"""ROS Kinetic field-session supervisor; uses the genuine RGB-D gate output."""
from __future__ import print_function
import argparse
import json
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime', default=os.path.expanduser('~/codex_method3_runtime'))
    p.add_argument('--duration', type=float, default=300.)
    p.add_argument('--distance', type=float, default=5.)
    p.add_argument('--start-stack', action='store_true')
    p.add_argument('--allow-unobserved', action='store_true',
                   help='Explicit field policy: unknown cells may be planned through; live obstacles still block')
    p.add_argument('--confirm-clearance', action='store_true')
    p.add_argument('--clearance-front', type=float, default=.30)
    p.add_argument('--clearance-half-width', type=float, default=.30)
    p.add_argument('--no-record', action='store_true')
    p.add_argument('--view-only', action='store_true', help='Start perception and visualization without motion')
    p.add_argument('--shutdown-stack', action='store_true',
                   help='Also close stack/viewer started by this session; default keeps them stationary for next test')
    args = p.parse_args()
    if args.allow_unobserved and args.confirm_clearance:
        p.error('Choose unknown-cell field policy or explicit clearance, not both')
    if not (0 < args.duration <= 300 and 0 < args.distance <= 5):
        p.error('Duration must be <=300 seconds and distance <=5 metres')
    import rospy
    from std_msgs.msg import Bool, String
    rospy.init_node('codex_field_session', anonymous=False, disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Field session requires live ROS time')
    ns = '/method3_motion'
    tools = os.path.dirname(os.path.abspath(__file__))
    os.environ.setdefault('DISPLAY', ':0')
    os.environ.setdefault('XAUTHORITY', os.path.expanduser('~/.Xauthority'))
    logdir = os.path.join(args.runtime, 'field_sessions', time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
    os.makedirs(logdir)
    owned, persistent, files, child = [], [], [], None
    stop = [False]
    def signal_stop(signum, frame):
        stop[0] = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal_stop)
    def launch(command, name):
        f = open(os.path.join(logdir, name+'.log'), 'w', 1)
        files.append(f)
        proc = subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT)
        owned.append(proc)
        if name != 'record':
            persistent.append(proc)
        return proc
    mission = rospy.Publisher(ns+'/test_mission', String, queue_size=1)
    lease = rospy.Publisher(ns+'/test_enable', Bool, queue_size=1)
    estop = rospy.Publisher(ns+'/test_estop', Bool, queue_size=1)
    trace = open(os.path.join(logdir, 'status.jsonl'), 'w', 1)
    files.append(trace)
    last = {}
    def status(msg, topic):
        value = msg.data
        if topic == 'perception_status':
            try:
                d = json.loads(value)
                value = {'reason': d.get('reason'), 'target_valid': d.get('target_valid')}
            except ValueError:
                pass
        if last.get(topic) != value:
            last[topic] = value
            line = json.dumps({'time': time.time(), 'topic': topic, 'value': value})
            trace.write(line+'\n')
            print(line)
    subscribers = [rospy.Subscriber(ns+'/'+t, String, status, callback_args=t)
                   for t in ('perception_status', 'follow_status', 'gate_status')]
    print('SESSION_LOG', logdir)
    try:
        code, message, state = rospy.get_master().getSystemState()
        if code != 1:
            raise RuntimeError(message)
        nodes = set(n for entries in state for topic, names in entries for n in names)
        if args.start_stack:
            launch_path = os.path.join(args.runtime, 'turtlebot_follow_current.launch')
            if args.allow_unobserved:
                tree = ET.parse(launch_path)
                for include in tree.getroot().findall('include'):
                    if 'follow.launch' not in include.get('file', ''):
                        continue
                    for name, value in (('commissioning_clearance', 'false'), ('allow_unobserved', 'true')):
                        element = next((e for e in include.findall('arg') if e.get('name') == name), None)
                        if element is None:
                            element = ET.SubElement(include, 'arg', {'name': name})
                        element.set('value', value)
                launch_path = os.path.join(logdir, 'field.launch')
                tree.write(launch_path)
                if (ns+'/rgbd_target' in nodes and
                        rospy.get_param(ns+'/move_base/global_costmap/track_unknown_space', True)):
                    # Required target node makes its enclosing roslaunch stop
                    # the complete follow stack. Base and camera remain active.
                    subprocess.check_call(['rosnode', 'kill', ns+'/rgbd_target'])
                    end = time.time()+8
                    while time.time()<end:
                        _, _, current = rospy.get_master().getSystemState()
                        names = set(n for entries in current for topic, names in entries for n in names)
                        if not any(n in names for n in (ns+'/rgbd_target', ns+'/move_base', ns+'/rgbd_velocity_gate')):
                            break
                        time.sleep(.1)
                    else:
                        raise RuntimeError('Previous follow stack did not shut down')
                    nodes.discard(ns+'/rgbd_target')
            if '/camera/driver' not in nodes:
                launch(['roslaunch', 'freenect_launch', 'freenect.launch',
                        'depth_registration:=true', 'ir_processing:=false',
                        'sw_registered_processing:=false', 'disparity_processing:=false',
                        'disparity_registered_processing:=false'], 'camera')
            if '/mobile_base' not in nodes:
                launch(['roslaunch', os.path.join(args.runtime, 'sensors.launch')], 'base')
            if ns+'/rgbd_target' not in nodes:
                launch(['roslaunch', launch_path], 'follow')
        if '/codex_live_follow_preview' not in nodes:
            launch(['rosrun', 'image_view', 'image_view', 'image:='+ns+'/target_preview',
                    '__name:=codex_live_follow_preview'], 'preview')
        if '/codex_method3_rviz' not in nodes:
            launch(['rosrun', 'rviz', 'rviz', '-d', os.path.join(tools, 'method3_field.rviz'),
                    '__name:=codex_method3_rviz'], 'pointcloud')
        print('Camera preview and RViz point cloud opened. Ctrl+C stops motion.')
        deadline = time.time()+25
        required = (ns+'/perception_status', ns+'/gate_status', '/odom', '/mobile_base/sensors/core')
        while not stop[0] and time.time() < deadline:
            code, _, state = rospy.get_master().getSystemState()
            pubs = dict(state[0])
            if code == 1 and all(pubs.get(t) for t in required):
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Required publishers unavailable; see startup logs')
        if stop[0]:
            return
        if args.view_only:
            print('VIEW_ONLY: no motion controller started')
            return
        if args.allow_unobserved and args.confirm_clearance:
            raise RuntimeError('Choose either unknown-cell field policy or explicit clearance, not both')
        if args.confirm_clearance:
            result = subprocess.call([sys.executable, os.path.join(tools, 'publish_commissioning_clearance.py'),
                                      '--confirm-clearance', '--front', str(args.clearance_front),
                                      '--rear', '.30', '--half-width', str(args.clearance_half_width)])
            if result != 0:
                raise RuntimeError('Clearance initialization failed')
        # Clearance is one fixed, expiring polygon. Never renew or move it
        # automatically: all subsequent path coverage comes from live sensors.
        if not args.no_record:
            launch(['rosbag', 'record', '-O', os.path.join(logdir, 'telemetry.bag')]+[
                '/odom', '/mobile_base/sensors/core', ns+'/perception_status',
                ns+'/follow_status', ns+'/gate_status', ns+'/target_pose',
                ns+'/target_valid', ns+'/cmd_vel_preview',
                ns+'/move_base/local_costmap/costmap',
                ns+'/move_base/DWAPlannerROS/local_plan',
                ns+'/move_base/DWAPlannerROS/global_plan', '/tf', '/tf_static'], 'record')
        f = open(os.path.join(logdir, 'motion.log'), 'w', 1)
        files.append(f)
        child = subprocess.Popen([sys.executable, '-u', os.path.join(tools, 'run_rgbd_bounded_trial.py'),
                                  '--namespace', ns, '--output', '/codex_diagnostics/disabled_velocity_input',
                                  '--field-test', '--duration', str(args.duration),
                                  '--distance', str(args.distance), '--yaw-limit', str(6.28),
                                  '--preflight', '30', '--resume-after-target-loss'],
                                 stdout=f, stderr=subprocess.STDOUT)
        deadline = time.time()+args.duration+32
        while not stop[0] and not rospy.is_shutdown() and child.poll() is None and time.time()<deadline:
            lease.publish(Bool(data=True))
            estop.publish(Bool(data=False))
            mission.publish(String(data='carrying'))
            time.sleep(.05)
    finally:
        for unused in range(10):
            lease.publish(Bool(data=False))
            estop.publish(Bool(data=True))
            mission.publish(String(data='idle'))
            time.sleep(.05)
        if child is not None:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            try:
                # Python 2 has no wait(timeout). Keep the supervisor bounded.
                end = time.time()+3
                while child.poll() is None and time.time()<end:
                    time.sleep(.05)
                if child.poll() is None:
                    child.kill()
                child.wait()
            finally:
                path = os.path.join(logdir, 'motion.log')
                if os.path.exists(path):
                    print(open(path).read())
        cleanup = owned if args.shutdown_stack else [proc for proc in owned if proc not in persistent]
        for proc in reversed(cleanup):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        end = time.time()+4
        while any(proc.poll() is None for proc in cleanup) and time.time()<end:
            time.sleep(.05)
        for proc in cleanup:
            if proc.poll() is None:
                proc.kill()
            proc.wait()
        for sub in subscribers:
            sub.unregister()
        rospy.signal_shutdown('field session finished')
        for f in files:
            f.close()
        print('SESSION_LOG', logdir)


if __name__ == '__main__':
    main()
