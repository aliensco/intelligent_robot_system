#!/usr/bin/env python
"""Check a deployed Method 3 bundle without starting ROS nodes or moving hardware."""
from __future__ import print_function
import argparse
import os
import platform
import re
import stat
import subprocess
import sys
import xml.etree.ElementTree as ET


ROS_PACKAGES = (
    'rgbd_following', 'rospy', 'roscpp', 'costmap_2d', 'pluginlib',
    'sensor_msgs', 'std_msgs', 'cv_bridge', 'message_filters', 'geometry_msgs',
    'nav_msgs', 'tf2_ros', 'actionlib', 'move_base_msgs', 'move_base', 'navfn',
    'dwa_local_planner', 'freenect_launch', 'freenect_camera', 'kobuki_node',
    'kobuki_msgs', 'image_view', 'rviz', 'rosbag', 'nodelet', 'diagnostic_aggregator',
)


class Report(object):
    def __init__(self):
        self.errors = 0

    def check(self, condition, message):
        print(('OK   ' if condition else 'FAIL ')+message)
        if not condition:
            self.errors += 1
        return bool(condition)

    def info(self, message):
        print('INFO '+message)


def run_readonly(argv):
    """All callers use fixed inspection commands, without a shell."""
    try:
        output = subprocess.check_output(argv, stderr=subprocess.STDOUT)
        return output.decode('utf-8', 'replace').strip(), None
    except (OSError, subprocess.CalledProcessError) as error:
        output = getattr(error, 'output', b'')
        if not isinstance(output, str):
            output = output.decode('utf-8', 'replace')
        return '', output.strip() or str(error)


def same_path(first, second):
    return os.path.normcase(os.path.realpath(first)) == os.path.normcase(os.path.realpath(second))


def check_runtime(root, yaml_module, report):
    runtime = os.path.join(root, 'runtime')
    report.info('Runtime: '+runtime)
    paths = dict((name, os.path.join(runtime, name)) for name in
                 ('sensors.launch', 'turtlebot_follow_current.launch',
                  'turtlebot_follow_current.yaml'))
    trees = {}
    for name, path in sorted(paths.items()):
        if not report.check(os.path.isfile(path), 'Runtime file: '+path):
            continue
        if name.endswith('.launch'):
            try:
                trees[name] = ET.parse(path)
            except (ET.ParseError, IOError) as error:
                report.check(False, name+' is not readable launch XML: '+str(error))
    tree = trees.get('turtlebot_follow_current.launch')
    if tree is not None:
        includes = [node for node in tree.getroot().iter('include')
                    if node.get('file', '').endswith('/launch/follow.launch')]
        if report.check(len(includes) == 1, 'Follow launch includes exactly one follow.launch'):
            settings = [node.get('value', '') for node in includes[0].findall('arg')
                        if node.get('name') == 'settings']
            if report.check(len(settings) == 1 and bool(settings[0]),
                            'Follow launch has one explicit settings argument'):
                value = settings[0]
                if '$(' in value:
                    report.check(False, 'Settings must be a plain path for the field runner: '+value)
                else:
                    resolved = value if os.path.isabs(value) else os.path.join(runtime, value)
                    report.check(os.path.isfile(resolved), 'Resolved settings: '+os.path.abspath(resolved))
                    report.check(same_path(resolved, paths['turtlebot_follow_current.yaml']),
                                 'Settings resolves to this bundle runtime profile')
                    if os.path.isabs(value):
                        report.info('Settings uses an absolute path; a relative filename is portable.')
    settings_path = paths['turtlebot_follow_current.yaml']
    if yaml_module is not None and os.path.isfile(settings_path):
        try:
            with open(settings_path, 'r') as stream:
                settings = yaml_module.safe_load(stream)
            report.check(isinstance(settings, dict) and isinstance(settings.get('perception'), dict),
                         'Settings YAML contains a perception configuration')
        except Exception as error:
            report.check(False, 'Settings YAML cannot be loaded: '+str(error))


def check_hardware(report):
    """Inspect USB enumeration and permissions; never open a device for I/O."""
    output, error = run_readonly(['lsusb'])
    if report.check(error is None, 'lsusb is available'+(': '+error if error else '')):
        camera = [line for line in output.splitlines()
                  if re.search(r'\b045e:(02ae|02bf)\b', line, re.I)]
        serial = [line for line in output.splitlines()
                  if re.search(r'\b0403:6001\b', line, re.I)]
        report.check(bool(camera), 'Kinect v1 camera USB: '+('; '.join(camera) or 'not found'))
        report.check(bool(serial), 'Kobuki FTDI USB family: '+('; '.join(serial) or 'not found'))
    device = '/dev/kobuki'
    if report.check(os.path.exists(device), 'Kobuki device link: '+device):
        try:
            info = os.stat(device)
            report.check(stat.S_ISCHR(info.st_mode), 'Kobuki resolves to character device: '+os.path.realpath(device))
            report.check(os.access(device, os.R_OK | os.W_OK),
                         'Current user has read/write permission on '+device)
        except OSError as error:
            report.check(False, 'Cannot inspect Kobuki device: '+str(error))
    report.info('Hardware inspection verifies enumeration/permissions, not camera frames or serial streaming.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        help='Bundle root containing tools/, runtime/, workspace/ and turtlebot/')
    parser.add_argument('--hardware', action='store_true',
                        help='Also inspect lsusb and /dev/kobuki permissions; no device writes')
    args = parser.parse_args(argv)
    report = Report()
    root = os.path.realpath(os.path.expanduser(args.root))
    report.info('Bundle: '+root)
    report.info('Python: '+platform.python_version()+' ('+sys.executable+')')
    report.info('Platform: '+platform.system()+' '+platform.machine())
    if not report.check(sys.version_info[:2] == (2, 7),
                        'ROS Kinetic runtime requires Python 2.7; --help also works with Python 3'):
        report.info('Source the bundle workspace, then run this check with python2.7.')
        return 1
    report.check(os.path.isdir(root), 'Bundle root exists')
    report.check(os.environ.get('ROS_DISTRO') == 'kinetic',
                 'ROS_DISTRO='+os.environ.get('ROS_DISTRO', '<unset>')+' (expected kinetic)')
    workspace = os.path.join(root, 'workspace')
    report.info('Workspace: '+workspace)
    report.check(os.path.isfile(os.path.join(workspace, 'devel', 'setup.bash')),
                 'Workspace devel/setup.bash exists')
    report.check(os.path.isfile(os.path.join(workspace, 'devel', 'lib',
                                           'librgbd_commissioning_clearance.so')),
                 'Compiled commissioning-clearance plugin exists')
    modules = {}
    for name in ('numpy', 'yaml', 'cv2', 'cv_bridge', 'rospy'):
        try:
            module = __import__(name)
            modules[name] = module
            report.check(True, '{} {}: {}'.format(name, getattr(module, '__version__', '(ROS module)'),
                                                 getattr(module, '__file__', '(built-in)')))
        except Exception as error:
            report.check(False, 'Cannot import '+name+': '+str(error))
    check_runtime(root, modules.get('yaml'), report)
    package_paths = {}
    available, error = run_readonly(['rospack', 'find', 'rospy'])
    if report.check(error is None, 'rospack can resolve rospy'+(': '+error if error else '')):
        package_paths['rospy'] = available
        for name in ROS_PACKAGES:
            if name == 'rospy':
                continue
            path, error = run_readonly(['rospack', 'find', name])
            if error:
                report.check(False, 'Missing ROS package '+name+': '+error)
            else:
                package_paths[name] = path
        report.info('ROS packages found: {}/{}'.format(len(package_paths), len(ROS_PACKAGES)))
    source = os.path.join(root, 'turtlebot', 'follow3', 'rgbd_following')
    found = package_paths.get('rgbd_following')
    if found:
        report.info('rospack rgbd_following: '+found)
        report.check(same_path(found, source), 'rgbd_following resolves to this bundle source: '+source)
    for name in ('move_base', 'freenect_camera', 'kobuki_node', 'rviz'):
        if name in package_paths:
            try:
                version = ET.parse(os.path.join(package_paths[name], 'package.xml')).findtext('version')
                report.info(name+' version: '+str(version))
            except (IOError, ET.ParseError):
                report.info(name+' version: unavailable')
    if args.hardware:
        check_hardware(report)
    else:
        report.info('Hardware checks skipped; no connected devices are required.')
    report.info('ROS master was not contacted; no node or motion controller was started.')
    print('RESULT: '+('FAIL ({} errors)'.format(report.errors) if report.errors else 'PASS'))
    return 1 if report.errors else 0


if __name__ == '__main__':
    sys.exit(main())
