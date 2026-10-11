"""Generated field profiles keep planner, gate and requested bounds consistent."""
from __future__ import division
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from run_rgbd_field_session import (prepare_field_launch, CAMERA_STREAMS,
                                    camera_stream_health, camera_node_names, field_paths)


class PortablePathsTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.mkdtemp(prefix='method3 team paths ')
        self.bundle = os.path.join(self.folder, 'team bundle')
        self.home = os.path.join(self.folder, 'another user')
        self.tools = os.path.join(self.bundle, 'tools')
        os.makedirs(self.tools)
        os.makedirs(self.home)

    def tearDown(self):
        shutil.rmtree(self.folder)

    def built_workspace(self, path):
        os.makedirs(os.path.join(path, 'devel'))
        with open(os.path.join(path, 'devel', 'setup.bash'), 'w') as stream:
            stream.write('set -- catkin_changed_arguments\n')

    def paths(self, env=None):
        return field_paths(self.tools, {} if env is None else env, self.home)

    def bash_path(self):
        bash = shutil.which('bash') if hasattr(shutil, 'which') else None
        if os.name == 'nt':
            bash = os.path.join(os.environ.get('ProgramFiles','C:/Program Files'),
                                'Git','bin','bash.exe')
        if not bash or not os.path.isfile(bash):
            self.skipTest('Bash is required for the shell entry-point test')
        return bash

    def test_new_bundle_defaults_are_relative_to_tools_not_username(self):
        self.assertEqual(self.paths(), (os.path.join(self.bundle,'workspace'),
                                        os.path.join(self.bundle,'runtime')))

    def test_built_bundle_wins_over_existing_legacy_install(self):
        self.built_workspace(os.path.join(self.home,'codex_method3_ws'))
        self.built_workspace(os.path.join(self.bundle,'workspace'))
        self.assertEqual(self.paths(), (os.path.join(self.bundle,'workspace'),
                                        os.path.join(self.bundle,'runtime')))

    def test_old_entry_point_falls_back_to_current_users_install(self):
        self.built_workspace(os.path.join(self.home,'codex_method3_ws'))
        self.assertEqual(self.paths(), (os.path.join(self.home,'codex_method3_ws'),
                                        os.path.join(self.home,'codex_method3_runtime')))

    def test_environment_overrides_preserve_spaces_and_expand_home(self):
        self.built_workspace(os.path.join(self.home,'codex_method3_ws'))
        workspace,runtime=self.paths(dict(METHOD3_WORKSPACE='~/team workspace',
                                          METHOD3_RUNTIME='~/team runtime'))
        self.assertEqual(workspace,os.path.join(self.home,'team workspace'))
        self.assertEqual(runtime,os.path.join(self.home,'team runtime'))

    def test_wrapper_uses_cli_runtime_for_lock_and_preserves_source_arguments(self):
        bash = self.bash_path()
        target=os.path.join(self.tools,'start_method3_field.sh')
        shutil.copyfile(os.path.join(ROOT,'tools','start_method3_field.sh'),target)
        self.built_workspace(os.path.join(self.bundle,'workspace'))
        fakebin=os.path.join(self.bundle,'fake bin')
        os.makedirs(fakebin)
        # Stop before the supervisor: this exercises shell argument/lock paths
        # without invoking ROS, a robot, or any background process.
        for name,body in (('timeout','exit 0\n'),
                          ('flock','printf "%s\\n" "$@" > "$METHOD3_TEST_TRACE"\n')):
            path=os.path.join(fakebin,name)
            with open(path,'w') as stream:
                stream.write('#!/usr/bin/env bash\n'+body)
            os.chmod(path,0o755)
        runtime=os.path.join(self.bundle,'chosen runtime')
        trace=os.path.join(self.bundle,'arguments.txt')
        env=os.environ.copy()
        env.pop('METHOD3_WORKSPACE',None)
        env['METHOD3_RUNTIME']=os.path.join(self.bundle,'unused env runtime')
        env['METHOD3_TEST_TRACE']=trace
        env['PATH']=fakebin+os.pathsep+env.get('PATH','')
        subprocess.check_call([bash,'tools/start_method3_field.sh','--runtime',runtime,
                               '--duration','42'],cwd=self.bundle,env=env)
        with open(trace) as stream:
            arguments=stream.read().splitlines()
        lock=next(a for a in arguments if a.endswith('/field_session.lock'))
        self.assertTrue(lock.endswith('/chosen runtime/field_session.lock'),lock)
        self.assertIn('--start-stack',arguments)
        self.assertEqual(arguments[-4:],['--runtime',runtime,'--duration','42'])
        self.assertNotIn('catkin_changed_arguments',arguments)
        self.assertFalse(os.path.exists(env['METHOD3_RUNTIME']))

    def test_wrapper_help_needs_no_workspace_or_ros_process(self):
        bash=self.bash_path()
        for name in ('start_method3_field.sh','run_rgbd_field_session.py'):
            shutil.copyfile(os.path.join(ROOT,'tools',name),os.path.join(self.tools,name))
        env=os.environ.copy()
        env['METHOD3_WORKSPACE']=os.path.join(self.bundle,'missing workspace')
        env['METHOD3_RUNTIME']=os.path.join(self.bundle,'unused runtime')
        env['METHOD3_PYTHON']=sys.executable
        for workspace_ready in (False, True):
            if workspace_ready:
                self.built_workspace(env['METHOD3_WORKSPACE'])
                with open(os.path.join(env['METHOD3_WORKSPACE'],'devel','setup.bash'),'w') as stream:
                    # A generated catkin setup may change shell arguments. Help
                    # must not execute setup or any ROS command even if present.
                    stream.write('echo "setup must not run for help" >&2\nexit 91\n')
            process=subprocess.Popen([bash,'tools/start_method3_field.sh','--help'],
                                     cwd=self.bundle,env=env,stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE,universal_newlines=True)
            stdout,stderr=process.communicate()
            self.assertEqual(process.returncode,0,stderr)
            self.assertIn('--max-speed',stdout)
            self.assertIn('--runtime',stdout)
            self.assertFalse(os.path.exists(env['METHOD3_RUNTIME']))


class Stamp(object):
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class FakeRos(object):
    class ROSException(Exception):
        pass

    class Time(object):
        @staticmethod
        def now():
            return Stamp(100.)

    def __init__(self, stream_stamps):
        self.stream_stamps = dict((name, list(values)) for name, values in stream_stamps.items())

    def wait_for_message(self, topic, message_type, timeout):
        values = self.stream_stamps.get(topic, [])
        if not values:
            raise self.ROSException('no message')
        message = type('Image', (object,), {})()
        message.header = type('Header', (object,), {})()
        message.header.stamp = Stamp(values.pop(0))
        return message


class CameraStartupTests(unittest.TestCase):
    def test_live_depth_with_missing_rectified_rgb_requires_recovery(self):
        ros = FakeRos({CAMERA_STREAMS[0][1]: [100.]})
        health = camera_stream_health(ros, object, timeout=.1)
        self.assertEqual(health['registered_depth'], 'fresh')
        self.assertNotEqual(health['rectified_rgb'], 'fresh')
        self.assertIn('/camera/rgb/image_rect_color', health['rectified_rgb'])

    def test_old_frames_are_not_renewed_by_new_receipt(self):
        ros = FakeRos(dict((topic, [99.]) for name, topic in CAMERA_STREAMS))
        health = camera_stream_health(ros, object, timeout=.1)
        self.assertTrue(all('source timestamp is stale' in value for value in health.values()))

    def test_fresh_rgb_and_depth_both_required(self):
        ros = FakeRos(dict((topic, [100.]) for name, topic in CAMERA_STREAMS))
        self.assertEqual(camera_stream_health(ros, object, timeout=.1),
                         dict(registered_depth='fresh', rectified_rgb='fresh'))

    def test_cleanup_includes_stranded_loaders_but_no_other_camera_or_base(self):
        nodes = ['/camera/driver', '/camera/camera_nodelet_manager', '/camera/rgb_rectify_color',
                 '/camera/depth_registered_rectify_depth', '/camera_nodelet_manager',
                 '/camera2/rgb_rectify_color', '/mobile_base', '/method3_motion/rgbd_target',
                 '/camera_base_link']
        self.assertEqual(camera_node_names(nodes), sorted(nodes[:5]))


class FieldProfileTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.mkdtemp(prefix='method3 settings with spaces ')
        self.source = os.path.join(self.folder, 'source.launch')
        self.output = os.path.join(self.folder, 'session')
        os.makedirs(self.output)
        self.config = dict(footprint_radius=.26, max_linear=.05, max_angular=.15,
                           follow_distance=.6, minimum_distance=.4,
                           perception=dict(identity=dict(references=[dict(label='enrolled_body')])) )
        with open(os.path.join(self.folder, 'source.yaml'), 'w') as destination:
            yaml.safe_dump(self.config, destination)
        with open(self.source, 'w') as destination:
            destination.write('''<launch>
  <param name="/method3_motion/move_base/DWAPlannerROS/max_vel_x" value="0.05" />
  <include file="$(find rgbd_following)/launch/follow.launch">
    <arg name="settings" value="source.yaml" />
    <arg name="allow_unobserved" value="true" />
    <arg name="commissioning_clearance" value="true" />
  </include>
  <param name="/method3_motion/move_base/DWAPlannerROS/max_vel_x" value="0.2" />
  <param name="/method3_motion/move_base/DWAPlannerROS/acc_lim_x" value="0.5" />
  <param name="/method3_motion/move_base/controller_frequency" value="10" />
</launch>''')

    def tearDown(self):
        shutil.rmtree(self.folder)

    def make(self, speed=.1, turn=.3, allow=False, clearance=False):
        path = prepare_field_launch(self.source, self.output, '/method3_motion',
                                    speed, turn, allow, clearance)
        root = ET.parse(path).getroot()
        params = dict((e.get('name').split('/move_base/')[-1], float(e.get('value')))
                      for e in root.findall('param'))
        include = root.find('include')
        args = dict((e.get('name'), e.get('value')) for e in include.findall('arg'))
        with open(args['settings']) as source:
            config = yaml.safe_load(source)
        return root, params, args, config

    def test_speed_and_vehicle_profile_match_in_both_map_policies(self):
        for allow in (False, True):
            root, params, args, config = self.make(.08, .25, allow)
            self.assertEqual(config['max_linear'], .08)
            self.assertEqual(config['max_angular'], .25)
            self.assertEqual(params['DWAPlannerROS/max_vel_x'], .08)
            self.assertEqual(params['DWAPlannerROS/max_trans_vel'], .08)
            self.assertEqual(params['DWAPlannerROS/max_rot_vel'], .25)
            self.assertEqual(args['allow_unobserved'], str(allow).lower())
            self.assertEqual(args['commissioning_clearance'], 'false')
            self.assertEqual(params['DWAPlannerROS/vx_samples'], 4)
            for name in ('global_costmap', 'local_costmap'):
                self.assertEqual(params[name+'/robot_radius'], config['footprint_radius'])
                self.assertEqual(params[name+'/footprint_padding'], 0.)
            self.assertEqual(config['perception'], self.config['perception'])
            self.assertEqual(config['minimum_distance'], .4)

    def test_minimum_velocities_are_reachable_for_small_requested_caps(self):
        unused, params, args, config = self.make(.02, .05)
        self.assertEqual(params['DWAPlannerROS/min_trans_vel'], .02)
        self.assertEqual(params['DWAPlannerROS/min_rot_vel'], .05)
        self.assertLessEqual(params['DWAPlannerROS/min_trans_vel'], params['DWAPlannerROS/max_trans_vel'])
        self.assertLessEqual(params['DWAPlannerROS/min_rot_vel'], params['DWAPlannerROS/max_rot_vel'])

    def test_first_dynamic_window_can_reach_both_velocity_minima(self):
        tree = ET.parse(self.source)
        for element in tree.getroot().findall('param'):
            if element.get('name').endswith('/controller_frequency'):
                element.set('value', '100')
        tree.write(self.source)
        unused, params, args, config = self.make()
        frequency = params['controller_frequency']
        self.assertLessEqual(params['DWAPlannerROS/min_trans_vel'], params['DWAPlannerROS/acc_lim_x']/frequency)
        self.assertLessEqual(params['DWAPlannerROS/min_rot_vel'], params['DWAPlannerROS/acc_lim_theta']/frequency)

    def test_duplicate_source_parameters_cannot_override_requested_cap(self):
        root, params, args, config = self.make()
        elements = [e for e in root.findall('param') if e.get('name').endswith('/DWAPlannerROS/max_vel_x')]
        self.assertEqual(len(elements), 1)
        self.assertGreater(list(root).index(elements[0]), list(root).index(root.find('include')))
        self.assertEqual(float(elements[0].get('value')), .1)
        # Session generation does not rewrite the source profile/calibration.
        self.assertEqual(len(ET.parse(self.source).getroot().findall('param')), 4)
        with open(os.path.join(self.folder, 'source.yaml')) as source:
            self.assertEqual(yaml.safe_load(source), self.config)

    def test_explicit_clearance_and_invalid_combinations(self):
        root, params, args, config = self.make(clearance=True)
        self.assertEqual(args['commissioning_clearance'], 'true')
        with self.assertRaises(ValueError):
            self.make(allow=True, clearance=True)
        for speed, turn in ((0., .3), (.101, .3), (.1, .301), (float('nan'), .3)):
            with self.assertRaises(ValueError):
                self.make(speed, turn)

    def test_missing_or_ambiguous_follow_include_is_reported(self):
        tree = ET.parse(self.source)
        tree.getroot().remove(tree.getroot().find('include'))
        tree.write(self.source)
        with self.assertRaises(ValueError):
            self.make()


if __name__ == '__main__':
    unittest.main()
