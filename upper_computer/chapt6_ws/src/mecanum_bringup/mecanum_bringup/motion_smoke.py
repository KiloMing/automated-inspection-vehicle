"""Bounded simulation-only motion check against independent Gazebo ground truth."""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Send motion commands in simulation')
    args = parser.parse_args()
    cases = [('forward', .1, 0., 0.), ('left', 0., .1, 0.),
             ('diagonal', .1, .1, 0.), ('rotate', 0., 0., .3)]
    if not args.execute:
        print('Dry run: 2 s per case at 20 Hz, zero between cases; simulation only.')
        print(json.dumps(cases))
        return
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = Node('mecanum_motion_smoke', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    pub = node.create_publisher(TwistStamped, '/mecanum_drive_controller/reference', 10)
    latest = {}
    def receive(key, msg):
        latest[key] = (msg, time.monotonic())
    subscriptions = [
        node.create_subscription(Odometry, '/ground_truth', lambda m: receive('truth', m), 10),
        node.create_subscription(Odometry, '/odom', lambda m: receive('odom', m), 10),
        node.create_subscription(Clock, '/clock', lambda m: receive('clock', m), 10),
    ]

    def fresh():
        return all(k in latest and time.monotonic()-latest[k][1] < .5
                   for k in ('truth', 'odom', 'clock'))

    def send(vx=0., vy=0., wz=0.):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        pub.publish(msg)

    def pose(key):
        p = latest[key][0].pose.pose
        q = p.orientation
        return p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def delta(start, end):
        x, y = end[0]-start[0], end[1]-start[1]
        return (math.cos(start[2])*x+math.sin(start[2])*y,
                -math.sin(start[2])*x+math.cos(start[2])*y,
                math.atan2(math.sin(end[2]-start[2]), math.cos(end[2]-start[2])))

    def pump(seconds, velocity=None):
        start_sim = node.get_clock().now().nanoseconds
        wall_deadline = time.monotonic()+max(10., seconds*10)
        while (node.get_clock().now().nanoseconds-start_sim)*1e-9 < seconds:
            if time.monotonic() > wall_deadline:
                raise RuntimeError('Simulation clock stalled or too slow')
            rclpy.spin_once(node, timeout_sec=.02)
            if not fresh():
                raise RuntimeError('Missing/stale simulation telemetry')
            if velocity is not None:
                send(*velocity)
            time.sleep(.03)

    results = []
    try:
        deadline = time.monotonic()+30
        while not fresh() or pub.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() > deadline:
                raise RuntimeError('Need /clock, /ground_truth, /odom and active controller')
        pump(.5, (0., 0., 0.))
        for name, vx, vy, wz in cases:
            start = {key: pose(key) for key in ('truth', 'odom')}
            pump(2., (vx, vy, wz))
            pump(.2, (0., 0., 0.))
            observed = {key: delta(start[key], pose(key)) for key in start}
            expected = (2*vx, 2*vy, 2*wz)
            for key, values in observed.items():
                for actual, target in zip(values, expected):
                    if abs(actual-target) > max(.06, abs(target)*.35):
                        raise RuntimeError(f'{name}: {key} {values}, expected {expected}')
            results.append({'case': name, 'observed': observed})
        # No explicit zero: verify controller reference_timeout actually stops motion.
        pump(.5, (.1, 0., 0.))
        pump(.7)
        start = pose('truth')
        pump(.5)
        stopped = delta(start, pose('truth'))
        if max(map(abs, stopped)) > .03:
            raise RuntimeError(f'Command timeout did not stop robot: {stopped}')
        print(json.dumps({'passed': results, 'timeout_stop_delta': stopped}, indent=2))
    finally:
        for _ in range(5):
            send()
            rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.shutdown()
