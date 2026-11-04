"""Bounded read-only raw C16 snapshots for checking actual scene motion."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True)
    p.add_argument('--seconds', type=float, default=180)
    a = p.parse_args()
    if not 10 <= a.seconds <= 300:
        raise ValueError('duration must be 10..300 seconds')
    out = Path(a.out); out.mkdir(exist_ok=False)
    rclpy.init(); node = Node('liujing_raw_motion_capture')
    start = time.monotonic(); last = 0.; count = 0
    meta = (out/'clouds.jsonl').open('x', buffering=1)
    poses = (out/'poses.jsonl').open('x', buffering=1)
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

    def cloud(m):
        nonlocal last, count
        now = time.monotonic()
        if now-last < 1:
            return
        fields = {f.name:f for f in m.fields}
        if any(fields[k].datatype != 7 for k in ('x','y','z')):
            raise ValueError('expected float32 XYZ')
        endian = '>' if m.is_bigendian else '<'
        xyz = np.stack([np.ndarray((m.height,m.width), dtype=endian+'f4',
            buffer=m.data, offset=fields[k].offset,
            strides=(m.row_step,m.point_step)).ravel() for k in ('x','y','z')],-1)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        name = f'{count:03}.npy'; np.save(out/name,xyz,allow_pickle=False)
        meta.write(json.dumps(dict(file=name, elapsed_s=now-start,
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            frame=m.header.frame_id, points=len(xyz)))+'\n')
        last=now; count+=1

    def pose(m, kind):
        v,q=m.pose.pose.position,m.pose.pose.orientation
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        poses.write(json.dumps(dict(kind=kind,elapsed_s=time.monotonic()-start,
            source_stamp=stamp,source_age_s=time.time()-stamp,
            position=[v.x,v.y,v.z],quaternion_xyzw=[q.x,q.y,q.z,q.w]))+'\n')

    subscriptions=[node.create_subscription(PointCloud2,'/point_cloud_raw',cloud,qos)]
    for kind,topic in [('raw','/odom_init/laser_odom'),('mapped','/mapping/odometry')]:
        subscriptions.append(node.create_subscription(Odometry,topic,lambda m,k=kind:pose(m,k),qos))
    try:
        while rclpy.ok() and time.monotonic()-start<a.seconds:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        (out/'summary.json').write_text(json.dumps(dict(clouds=count,elapsed_s=time.monotonic()-start,
            scope='Raw scene-motion evidence; not calibration acceptance'),indent=2))
        meta.close(); poses.close(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
