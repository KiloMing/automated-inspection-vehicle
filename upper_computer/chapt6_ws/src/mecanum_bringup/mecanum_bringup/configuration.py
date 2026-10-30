"""Shared geometry loading and controller configuration generation."""
from pathlib import Path
import yaml
from .kinematics import Geometry


def controller_config(template, geometry_file):
    geometry = yaml.safe_load(Path(geometry_file).read_text())
    dimensions = Geometry(geometry['radius'], geometry['half_length'], geometry['half_width'])
    config = yaml.safe_load(Path(template).read_text())
    config['controller_manager']['ros__parameters']['use_sim_time'] = True
    params = config['mecanum_drive_controller']['ros__parameters']
    params['kinematics'] = {
        'wheels_radius': dimensions.radius,
        'sum_of_robot_center_projection_on_X_Y_axis': dimensions.lever,
        'base_frame_offset': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
    }
    return config
