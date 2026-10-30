import unittest
from mecanum_bringup.sdf_validation import parse_sdf


class SdfValidationTest(unittest.TestCase):
    def test_gazebo_prefix_without_xml_namespace(self):
        root = parse_sdf('''<?xml version="1.0"?><sdf version="1.9"><model name="robot">
        <fdir1 gz:expressed_in="base_link">1 -1 0</fdir1></model></sdf>''')
        element = root.find('model/fdir1')
        self.assertEqual(element.get('{http://gazebosim.org/schema}expressed_in'), 'base_link')

    def test_missing_frame_is_not_invented(self):
        root = parse_sdf('<sdf><model><fdir1>1 -1 0</fdir1></model></sdf>')
        self.assertIsNone(root.find('model/fdir1').get('{http://gazebosim.org/schema}expressed_in'))
