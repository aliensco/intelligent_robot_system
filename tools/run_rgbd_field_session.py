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
import yaml


CAMERA_STREAMS = (('registered_depth', '/camera/depth_registered/hw_registered/image_rect'),
                  ('rectified_rgb', '/camera/rgb/image_rect_color'))


def wait_for_fresh_message(ros, topic, message_type, timeout, max_age=.5):
    """A live publisher/receipt does not prove that source frames are advancing."""
    deadline = time.time()+timeout
    reason = 'no new message'
    while time.time() < deadline:
        try:
            msg = ros.wait_for_message(topic, message_type, timeout=max(.01, deadline-time.time()))
        except ros.ROSException:
            raise RuntimeError(topic+': '+reason+' within '+str(timeout)+' s')
        stamp = msg.header.stamp.to_sec()
        age = ros.Time.now().to_sec()-stamp
        if stamp > 0 and 0 <= age <= max_age:
            return msg
        reason = 'source timestamp is stale or invalid (age '+str(round(age, 3))+' s)'
    raise RuntimeError(topic+': '+reason)


def camera_stream_health(ros, image_type, timeout):
    health = {}
    for name, topic in CAMERA_STREAMS:
        try:
            wait_for_fresh_message(ros, topic, image_type, timeout)
            health[name] = 'fresh'
        except RuntimeError as error:
            health[name] = str(error)
    return health


def camera_node_names(nodes):
    """Only the current camera namespace and its legacy global manager."""
    return sorted(n for n in nodes if n.startswith('/camera/') or n == '/camera_nodelet_manager')


def prepare_field_launch(source_path, logdir, namespace, max_speed, max_turn,
                         allow_unobserved=False, commissioning_clearance=False):
    """Generate one coherent planner/gate profile; independent of ROS/hardware."""
    if not (0 < max_speed <= .10 and 0 < max_turn <= .30):
        raise ValueError('Maximum speed must be <=0.10 m/s and turn rate <=0.30 rad/s')
    if allow_unobserved and commissioning_clearance:
        raise ValueError('Choose unknown-cell policy or explicit clearance, not both')
    ns = namespace.rstrip('/')
    tree = ET.parse(source_path)
    root = tree.getroot()
    includes = [e for e in root.findall('include') if 'follow.launch' in e.get('file', '')]
    if len(includes) != 1:
        raise ValueError('Runtime launch must include exactly one follow.launch')
    include = includes[0]
    for name, value in (('commissioning_clearance', commissioning_clearance),
                        ('allow_unobserved', allow_unobserved)):
        element = next((e for e in include.findall('arg') if e.get('name') == name), None)
        if element is None:
            element = ET.SubElement(include, 'arg', {'name': name})
        element.set('value', 'true' if value else 'false')
    settings_arg = next((e for e in include.findall('arg') if e.get('name') == 'settings'), None)
    if settings_arg is None or not settings_arg.get('value'):
        raise ValueError('Runtime follow.launch needs an explicit settings file')
    settings_source = settings_arg.get('value')
    if not os.path.isabs(settings_source):
        settings_source = os.path.join(os.path.dirname(os.path.abspath(source_path)), settings_source)
    with open(settings_source) as source:
        field_settings = yaml.safe_load(source)
    field_settings.update(detour_max_extra_distance=.6, navigation_retry_delay=.8, detour_goal_hold=8.,
                          max_linear=max_speed, max_angular=max_turn)
    settings_path = os.path.join(logdir, 'field_settings.yaml')
    with open(settings_path, 'w') as destination:
        yaml.safe_dump(field_settings, destination, default_flow_style=False)
    settings_arg.set('value', settings_path)

    def existing_number(name, default):
        elements = [e for e in root.findall('param') if e.get('name') == name]
        return float(elements[-1].get('value')) if elements else default

    def override(name, value):
        # Put overrides after the include and eliminate duplicates so imported
        # defaults or an old repeated param cannot silently win over the CLI.
        for element in list(root.findall('param')):
            if element.get('name') == name:
                root.remove(element)
        ET.SubElement(root, 'param', dict(name=name, value=str(value),
                                       type='int' if isinstance(value, int) else 'double'))

    planner = ns+'/move_base/'
    frequency = existing_number(planner+'controller_frequency', 10.)
    acceleration = existing_number(planner+'DWAPlannerROS/acc_lim_x', .3)
    if frequency <= 0 or acceleration <= 0:
        raise ValueError('Controller frequency and forward acceleration must be positive')
    # Minimum velocities must fit both requested caps and the first DWA window.
    # Four samples include zero and intermediate curved trajectories at .10 m/s.
    params = dict(min_vel_x=0., max_rot_vel=max_turn, max_vel_x=max_speed,
                  max_trans_vel=max_speed, min_trans_vel=min(.03, max_speed, acceleration/frequency),
                  min_rot_vel=min(.30, max_turn, 3.0/frequency),
                  xy_goal_tolerance=.05, yaw_goal_tolerance=.25,
                  sim_time=2., vx_samples=4, acc_lim_theta=3.0)
    for name, value in sorted(params.items()):
        override(planner+'DWAPlannerROS/'+name, value)
    # The measured radius already includes its margin. DWA and the gate use the
    # same physical envelope; user speed choices never shrink the vehicle.
    for costmap in ('global_costmap', 'local_costmap'):
        override(planner+costmap+'/robot_radius', float(field_settings['footprint_radius']))
        override(planner+costmap+'/footprint_padding', 0.)
    launch_path = os.path.join(logdir, 'field.launch')
    tree.write(launch_path)
    return launch_path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime', default=os.path.expanduser('~/codex_method3_runtime'))
    p.add_argument('--duration', type=float, default=300.)
    p.add_argument('--distance', type=float, default=5.)
    p.add_argument('--max-speed', type=float, default=.10,
                   help='Maximum straight speed, metres/second (0 < speed <= 0.10)')
    p.add_argument('--max-turn', type=float, default=.30,
                   help='Maximum yaw rate, radians/second (0 < rate <= 0.30)')
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
    if not (0 < args.max_speed <= .10 and 0 < args.max_turn <= .30):
        p.error('Maximum speed must be <=0.10 m/s and turn rate <=0.30 rad/s')
    import rospy
    import rosnode
    from std_msgs.msg import Bool, String
    from kobuki_msgs.msg import SensorState
    from sensor_msgs.msg import Image
    rospy.init_node('codex_field_session', anonymous=False, disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Field session requires live ROS time')
    ns = '/method3_motion'
    tools = os.path.dirname(os.path.abspath(__file__))
    os.environ.setdefault('DISPLAY', ':0')
    os.environ.setdefault('XAUTHORITY', os.path.expanduser('~/.Xauthority'))
    # VMware's vmwgfx OpenGL driver can crash RViz on this Ubuntu image.
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
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
        camera_nodes = camera_node_names(nodes)
        for node in ('/mobile_base', '/camera/driver', ns+'/rgbd_target',
                     '/codex_live_follow_preview', '/codex_method3_rviz'):
            if node in nodes and not rosnode.rosnode_ping(node, max_count=1, verbose=False):
                nodes.discard(node)
        if args.start_stack:
            # A registered node can survive a USB timeout without producing data.
            # Recover each driver once before deciding whether to reuse it.
            for name, topic, msgtype, driver_nodes in (
                    ('base', '/mobile_base/sensors/core', SensorState,
                     ('/mobile_base', '/mobile_base_nodelet_manager', '/diagnostic_aggregator')),):
                driver = '/mobile_base'
                if driver not in nodes:
                    continue
                try:
                    rospy.wait_for_message(topic, msgtype, timeout=2.)
                except rospy.ROSException:
                    print('Restarting stalled '+name+' driver: no live data')
                    subprocess.call(['rosnode', 'kill']+list(driver_nodes))
                    time.sleep(2.)
                    nodes.discard(driver)
            if camera_nodes:
                health = camera_stream_health(rospy, Image, timeout=2.)
                if any(value != 'fresh' for value in health.values()):
                    print('Rebuilding Kinect nodelets: '+json.dumps(health, sort_keys=True))
                    # A manager restart can strand live loader nodes that only
                    # publish /rosout. Restart every camera loader, not just the
                    # manager; otherwise roslaunch may reuse an empty RGB chain.
                    subprocess.call(['rosnode', 'kill']+camera_nodes)
                    end = time.time()+8.
                    while time.time() < end:
                        code, _, current = rospy.get_master().getSystemState()
                        current_names = set(n for entries in current for topic, names in entries for n in names)
                        remaining = camera_node_names(current_names)
                        if code == 1 and not remaining:
                            break
                        time.sleep(.1)
                    else:
                        raise RuntimeError('Camera nodelets did not shut down: '+', '.join(remaining))
                    nodes.difference_update(camera_nodes)
                    nodes.discard('/camera/driver')
                else:
                    # Both measured streams suffice even if the driver ping
                    # briefly failed while its nodelet callbacks were busy.
                    nodes.add('/camera/driver')
            launch_path = prepare_field_launch(
                os.path.join(args.runtime, 'turtlebot_follow_current.launch'), logdir, ns,
                args.max_speed, args.max_turn, args.allow_unobserved, args.confirm_clearance)
            if ns+'/rgbd_target' in nodes:
                # A new session rebuilds follow state under either map policy.
                # Required target shutdown leaves base/camera drivers running.
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
                # Old unregistered nodelet managers can still own the serial
                # port. Terminate only verified Kobuki managers using that port.
                port = os.path.realpath('/dev/kobuki')
                owners = subprocess.Popen(['fuser', port], stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE)
                output, unused = owners.communicate()
                for pid in output.decode('ascii', 'ignore').split():
                    if not pid.isdigit():
                        continue
                    try:
                        command = open('/proc/'+pid+'/cmdline', 'rb').read()
                        if (b'/nodelet/nodelet' not in command or
                                b'__name:=mobile_base_nodelet_manager' not in command):
                            raise RuntimeError('Base serial port is occupied by another application')
                        print('Closing residual Kobuki manager PID '+pid)
                        os.kill(int(pid), signal.SIGTERM)
                        end = time.time()+3.
                        while os.path.exists('/proc/'+pid) and time.time()<end:
                            time.sleep(.1)
                        if os.path.exists('/proc/'+pid):
                            os.kill(int(pid), signal.SIGKILL)
                    except IOError:
                        pass
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
        try:
            rospy.wait_for_message('/mobile_base/sensors/core', SensorState, timeout=15.)
        except rospy.ROSException:
            raise RuntimeError('Base has no sensor messages; see base.log')
        print('Checking fresh rectified RGB and registered depth frames...')
        health = camera_stream_health(rospy, Image, timeout=15.)
        if any(value != 'fresh' for value in health.values()):
            raise RuntimeError('Kinect streams unavailable after startup: '+json.dumps(health, sort_keys=True))
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
                '/codex_diagnostics/disabled_velocity_input',
                ns+'/follow_status', ns+'/gate_status', ns+'/target_pose',
                ns+'/target_valid', ns+'/cmd_vel_preview',
                ns+'/move_base/local_costmap/costmap',
                ns+'/move_base/global_costmap/costmap',
                ns+'/move_base/DWAPlannerROS/local_plan',
                ns+'/move_base/DWAPlannerROS/global_plan', '/tf', '/tf_static'], 'record')
        f = open(os.path.join(logdir, 'motion.log'), 'w', 1)
        files.append(f)
        child = subprocess.Popen([sys.executable, '-u', os.path.join(tools, 'run_rgbd_bounded_trial.py'),
                                  '--namespace', ns, '--output', '/codex_diagnostics/disabled_velocity_input',
                                  '--field-test', '--duration', str(args.duration),
                                  '--distance', str(args.distance), '--yaw-limit', str(6.28),
                                  '--max-linear', str(args.max_speed), '--max-angular', str(args.max_turn),
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
