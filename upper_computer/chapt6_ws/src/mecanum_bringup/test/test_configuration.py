from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml
import xacro
from mecanum_bringup.configuration import controller_config

ROOT = Path(__file__).resolve().parents[1]


class ConfigurationTest(unittest.TestCase):
    def test_omni_pipeline(self):
        data = yaml.safe_load((ROOT/'config/nav2_omni.yaml').read_text())
        c = data['controller_server']['ros__parameters']
        self.assertEqual(c['FollowPath']['motion_model'], 'Omni')
        self.assertLess(c['min_y_velocity_threshold'], 0.01)
        self.assertTrue(c['enable_stamped_cmd_vel'])
        self.assertNotIn('PreferForwardCritic', c['FollowPath']['critics'])
        self.assertEqual(data['amcl']['ros__parameters']['robot_model_type'], 'nav2_amcl::OmniMotionModel')
        s = data['velocity_smoother']['ros__parameters']
        self.assertGreater(s['max_velocity'][1], 0)
        self.assertLess(s['min_velocity'][1], 0)
        self.assertGreater(s['max_accel'][1], 0)
        self.assertLess(s['max_decel'][1], 0)
        collision = data['collision_monitor']['ros__parameters']
        self.assertEqual(collision['cmd_vel_out_topic'], '/mecanum_drive_controller/reference')
        for node in ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor'):
            self.assertTrue(data[node]['ros__parameters']['enable_stamped_cmd_vel'])
        self.assertEqual(collision['base_frame_id'], 'base_link')

    def test_shared_geometry_and_four_interfaces(self):
        config = controller_config(ROOT/'config/controllers.yaml', ROOT/'config/geometry.yaml')
        params = config['mecanum_drive_controller']['ros__parameters']
        geom = yaml.safe_load((ROOT/'config/geometry.yaml').read_text())
        self.assertEqual(params['kinematics']['wheels_radius'], geom['radius'])
        self.assertAlmostEqual(params['kinematics']['sum_of_robot_center_projection_on_X_Y_axis'], geom['half_length']+geom['half_width'])
        self.assertGreater(params['reference_timeout'], 0)
        model = ET.fromstring(xacro.process_file(str(ROOT/'urdf/mecanum.urdf.xacro'), mappings={
            'geometry_file': str(ROOT/'config/geometry.yaml'), 'controllers_file': '/tmp/test.yaml',
            'simulation': 'true'}).toxml())
        joints = model.findall('ros2_control/joint')
        self.assertEqual(len(joints), 4)
        for joint in joints:
            self.assertEqual(joint.find('command_interface').get('name'), 'velocity')
            self.assertIn(joint.get('name'), params.values())
        for name, sign in [('front_left', '-1'), ('front_right', '1'), ('rear_left', '1'), ('rear_right', '-1')]:
            joint = model.find(f"joint[@name='{name}_wheel_joint']")
            self.assertEqual(joint.find('axis').get('xyz'), '0 1 0')
            friction = model.find(f"gazebo[@reference='{name}_wheel']/collision/surface/friction/ode/fdir1")
            self.assertEqual(friction.text, f'1 {sign} 0')
            self.assertEqual(friction.get('{http://gazebosim.org/schema}expressed_in'), 'base_link')

    def test_real_model_has_no_simulation_controller(self):
        model = ET.fromstring(xacro.process_file(str(ROOT/'urdf/mecanum.urdf.xacro'), mappings={
            'geometry_file': str(ROOT/'config/geometry.yaml'), 'simulation': 'false'}).toxml())
        self.assertIsNone(model.find('ros2_control'))
        self.assertIsNone(model.find('gazebo'))
        self.assertIsNotNone(model.find("joint[@name='laser_joint']"))
