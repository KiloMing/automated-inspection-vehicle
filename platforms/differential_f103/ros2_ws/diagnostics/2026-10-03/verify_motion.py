import json
import math
import os
import subprocess
import time
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray

os.environ['GZ_IP'] = '127.0.0.1'
rclpy.init()
node = rclpy.create_node('codex_verify_motion')
states = []
clocks = []
node.create_subscription(JointState, '/joint_states', states.append, 10)
node.create_subscription(Clock, '/clock', lambda m: clocks.append(
    m.clock.sec + m.clock.nanosec * 1e-9), 10)
pub = node.create_publisher(Float64MultiArray, '/codex_verify_velocity/commands', 10)


def spin(seconds, command=None):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if command is not None:
            pub.publish(Float64MultiArray(data=command))
        rclpy.spin_once(node, timeout_sec=0.05)


def pose():
    result = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/my_world/dynamic_pose/info',
         '-n', '1', '--json-output'],
        capture_output=True, text=True, timeout=12, check=True)
    data = json.loads(result.stdout)
    return next(p for p in data['pose'] if p['name'] == 'second_robot')


report = {}
try:
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and not (states and clocks):
        spin(0.2, [0.0, 0.0])
    assert states and clocks, 'Joint states and ROS clock must arrive'
    start_pose = pose()
    start = states[-1]
    offset = len(states)
    spin(4, [1.0, 1.0])
    end_pose = pose()
    moving = states[offset:]
    assert moving, 'No states during command'
    for name in ['left_joint', 'right_joint']:
        i = start.name.index(name)
        j = states[-1].name.index(name)
        velocities = [m.velocity[m.name.index(name)] for m in moving]
        delta = states[-1].position[j] - start.position[i]
        assert max(velocities) > 0.5, (name, 'wheel did not turn')
        assert delta > 0.1, (name, 'position did not advance')
        report[name] = {'max_velocity_rad_s': max(velocities),
                        'position_delta_rad': delta}
    displacement = math.hypot(
        end_pose['position'].get('x', 0) - start_pose['position'].get('x', 0),
        end_pose['position'].get('y', 0) - start_pose['position'].get('y', 0))
    assert displacement > 0.005, 'Robot did not translate'
    report['translation_m'] = displacement
    report['clock_advanced_s'] = clocks[-1] - clocks[0]
    assert report['clock_advanced_s'] > 0
finally:
    spin(2, [0.0, 0.0])
    if states:
        last = states[-1]
        report['stopped_velocity_rad_s'] = dict(zip(last.name, last.velocity))
    Path('diagnostics/2026-10-03/motion.json').write_text(
        json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
for name in ['left_joint', 'right_joint']:
    assert abs(report['stopped_velocity_rad_s'][name]) < 0.1
print(json.dumps(report, indent=2))
