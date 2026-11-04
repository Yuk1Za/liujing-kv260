"""Bounded read-only pose/display evidence for a manually guided motion check.

Does not command the chassis, start sensors, change calibration, or certify it.
Run with the live session's ROS_DOMAIN_ID after the static measurement ends.
"""
import argparse
import json
import math
import time
import urllib.request
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Probe(Node):
    def __init__(self, out, start):
        super().__init__('liujing_movement_probe')
        self.out, self.start = out, start
        self.log = (out/'poses.jsonl').open('x')
        self.counts = {'raw': 0, 'mapped': 0}
        self.subscriptions_owned = []
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        for kind, topic in [('raw', '/odom_init/laser_odom'), ('mapped', '/mapping/odometry')]:
            self.subscriptions_owned.append(self.create_subscription(
                Odometry, topic, lambda m, k=kind: self.record(k, m), qos))

    def record(self, kind, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite pose')
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        self.log.write(json.dumps(dict(kind=kind, elapsed_s=time.monotonic()-self.start,
            source_stamp=stamp, source_age_s=time.time()-stamp,
            frame=message.header.frame_id, child=message.child_frame_id,
            position=values[:3], quaternion_xyzw=values[3:]))+'\n')
        self.counts[kind] += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--port', type=int, default=8090)
    a = parser.parse_args()
    if not 10 <= a.seconds <= 900:
        raise ValueError('bounded duration must be 10..900 seconds')
    out = Path(a.out); out.mkdir(exist_ok=False)
    rclpy.init(); start = time.monotonic(); probe = Probe(out, start)
    status = (out/'display.jsonl').open('x'); next_poll = start
    try:
        while rclpy.ok() and time.monotonic()-start < a.seconds:
            rclpy.spin_once(probe, timeout_sec=.02)
            if time.monotonic() >= next_poll:
                next_poll = time.monotonic()+1
                row = {'elapsed_s': time.monotonic()-start}
                try:
                    with urllib.request.urlopen(f'http://127.0.0.1:{a.port}/status', timeout=.3) as response:
                        row['status'] = json.load(response)
                except Exception as exc:
                    row['error'] = str(exc)
                status.write(json.dumps(row)+'\n'); status.flush(); probe.log.flush()
    finally:
        (out/'summary.json').write_text(json.dumps(dict(scope='Known-motion diagnostic, not acceptance',
            counts=probe.counts, elapsed_s=time.monotonic()-start), indent=2))
        status.close(); probe.log.close(); probe.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
