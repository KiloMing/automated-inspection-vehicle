#!/usr/bin/env python3
"""Check actual Harmonic conversion, including the custom friction frame."""
import subprocess
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory

share = Path(get_package_share_directory('mecanum_bringup'))
with tempfile.TemporaryDirectory() as folder:
    urdf = Path(folder)/'robot.urdf'
    urdf.write_text(xacro.process_file(str(share/'urdf/mecanum.urdf.xacro'), mappings={
        'geometry_file': str(share/'config/geometry.yaml'),
        'simulation': 'true', 'controllers_file': str(share/'config/controllers.yaml'),
    }).toxml())
    result = subprocess.run(['gz', 'sdf', '-p', str(urdf)], check=True, capture_output=True, text=True)
    root = ET.fromstring(result.stdout)
    for name, sign in [('front_left', -1), ('front_right', 1), ('rear_left', 1), ('rear_right', -1)]:
        link = root.find(f"model/link[@name='{name}_wheel']")
        assert link is not None, name
        direction = link.find('collision/surface/friction/ode/fdir1')
        assert direction is not None, name
        assert direction.get('{http://gazebosim.org/schema}expressed_in') == 'base_link', ET.tostring(direction)
        assert tuple(map(float, direction.text.split())) == (1., float(sign), 0.)
    print('Converted SDF preserves all four chassis-relative friction directions')
