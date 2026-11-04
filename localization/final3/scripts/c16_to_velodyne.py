#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
c16_to_velodyne.py —— 镭神 C16 驱动输出 → lo_sam 输入格式

lo_sam 的 feature_extract 只读 x/y/z/intensity 四字段（intensity 会被
allocate_rangeimage 覆写为 scanID+relTime），因此转换只需：
  1. topic 重映射（默认 /lslidar_point_cloud → /velodyne_points）
  2. 清理 NaN / 零点（C16 驱动偶发无效点）
  3. 时间戳标准化（可选：驱动时间戳 vs 接收时刻）

用法（板端）：
  python3 c16_to_velodyne.py --in /lslidar_point_cloud --out /velodyne_points
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class C16ToVelodyne(Node):
    def __init__(self, in_topic, out_topic, min_range):
        super().__init__('c16_to_velodyne')
        self.min_range = min_range
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.get_logger().info(f'{in_topic} -> {out_topic} (min_range={min_range})')

    def cb(self, msg: PointCloud2):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        # C16 驱动坐标系是 x 前向；VLP16/lo_sam 也按 x 前向处理，无需旋转
        r2 = pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2
        keep = r2 > self.min_range ** 2
        pts = pts[keep]
        n = pts.shape[0]
        if n == 0:
            return
        data = np.zeros((n, 4), dtype=np.float32)
        data[:, :3] = pts
        # intensity 占位：allocate_rangeimage 会覆写，置 0 即可
        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, n
        out.fields = self.fields
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * n
        out.is_dense = True
        out._data = data.tobytes()
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', default='/lslidar_point_cloud', dest='in_topic')
    ap.add_argument('--out', default='/velodyne_points')
    ap.add_argument('--min-range', type=float, default=0.3)
    a = ap.parse_args()
    rclpy.init()
    node = C16ToVelodyne(a.in_topic, a.out_topic, a.min_range)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
