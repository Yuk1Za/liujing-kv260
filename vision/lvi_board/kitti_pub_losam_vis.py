#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kitti_pub_losam_vis.py —— 在 lo_sam 自带 publisher 上加两件事：帧号 + 抽稀话题。

为什么不用我们自己的 kitti_pub.py
----------------------------------
实测对照（同一颗融合位流、同样 60 帧）：
  · lo_sam 自带 publisher（**KITTI 时间戳**）  → 后端轨迹正常移动，x 4.2→33.3 m / y 2.5→73.8 m
  · 我们的 kitti_pub.py（**wall clock 时间戳**）→ 后端轨迹**恒为 0**（x∈[0,0]）
⇒ 差异不在字段（两边都是 x,y,z,intensity float32 / point_step 16），而在 **header.stamp**：
  pldataprocess 是"四路同步 PD_SYNC=fix"、laserOdometry 有 `SCAN_PERIOD=0.1` 的假设，
  用板子的 wall clock（无 RTC，且 0.4 Hz 一拍）下游就退化成静止。
  同时自带 publisher 的 RELIABLE depth=10 也不会像 depth=1 那样被大消息卡住。

做了什么（只在发布侧，下游零改动）
----------------------------------
1. `frame_id` 变成 `velodyne:<idx>`（板上已核对：下游**不读**输入帧的 frame_id，
   只在自己发出的消息里写 "camera_init"/"laser_odom"）⇒ 视觉节点靠它定位图像，雷达无感；
2. 另发一条 `--vis_stride` 抽稀的 `/velodyne_points_vis`（BEST_EFFORT）给视觉节点，
   避免它再吃一遍 976 KB；
3. 时间戳仍用 **KITTI timestamps.txt**（这是后端能动的关键）。

用法：
    python3 kitti_pub_losam_vis.py <velodyne_data_dir> <timestamps.txt> [freq] [max_frames]
环境变量：
    VIS_STRIDE=8  VIS_TOPIC=/velodyne_points_vis  TAG_INDEX=1(0=frame_id 不带帧号)
"""
import os
import sys

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_bin_file(bin_path):
    return np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)


def pointcloud2_msg(points, frame_id='velodyne'):
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = True
    msg._data = points.astype(np.float32).tobytes()
    return msg


def kitti_timestamp_to_ros_time(ts_str):
    dt = datetime.strptime(ts_str[:26], "%Y-%m-%d %H:%M:%S.%f")
    t = dt.timestamp()
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


class KittiPubVis(Node):
    def __init__(self, data_path, ts_file, frame_id='velodyne',
                 freq=10.0, max_frames=0):
        super().__init__('kitti_pointcloud_publisher')
        self.data_path = data_path
        self.frame_id = frame_id
        self.max_frames = int(max_frames)
        self.vis_stride = int(os.environ.get('VIS_STRIDE', '8'))
        self.tag_index = os.environ.get('TAG_INDEX', '1') == '1'

        # 主话题：与自带 publisher 一致（RELIABLE depth=10）——lo_sam 的订阅方要 RELIABLE
        q_pc = QoSProfile(depth=10)
        q_pc.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(PointCloud2, '/velodyne_points', q_pc)
        # 视觉话题：BEST_EFFORT，抽稀后 ~240 KB，不给可靠重传拖住
        q_vis = QoSProfile(depth=1)
        q_vis.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub_vis = self.create_publisher(
            PointCloud2, os.environ.get('VIS_TOPIC', '/velodyne_points_vis'), q_vis)

        self.files = sorted(os.listdir(data_path))
        with open(ts_file, 'r') as f:
            self.timestamps = [x.strip() for x in f.readlines()]
        assert len(self.files) == len(self.timestamps), \
            f'点云/时间戳数量不匹配: {len(self.files)} vs {len(self.timestamps)}'

        self.index = 0
        self.timer = self.create_timer(1.0 / freq, self.tick)
        self.get_logger().info(
            f'回放 {len(self.files)} 帧 @ {freq} Hz  vis_stride={self.vis_stride} '
            f'tag_index={self.tag_index} max_frames={self.max_frames}')

    def tick(self):
        if self.index >= len(self.files) or \
                (self.max_frames and self.index >= self.max_frames):
            self.get_logger().info(f'回放结束（{self.index} 帧），关闭 publisher')
            self.timer.cancel()
            import time as _t
            _t.sleep(1.0)
            os._exit(0)          # 硬退出：rclpy 收尾会卡在 DDS 拆除并忽略 SIGTERM
            return

        idx = self.index
        pts = read_bin_file(os.path.join(self.data_path, self.files[idx]))
        fid = f'{self.frame_id}:{idx}' if self.tag_index else self.frame_id
        stamp = kitti_timestamp_to_ros_time(self.timestamps[idx])

        if self.vis_stride > 1:
            v = pts[::self.vis_stride] if self.vis_stride > 1 else pts
            mv = pointcloud2_msg(v, frame_id=fid)
            mv.header.stamp = stamp
            self.pub_vis.publish(mv)

        m = pointcloud2_msg(pts, frame_id=fid)
        m.header.stamp = stamp
        self.pub.publish(m)

        if idx % 10 == 0:
            self.get_logger().info(f'frame {idx}  {self.files[idx]}  {self.timestamps[idx]}')
        self.index += 1


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:                       # 剔除 launch 注入的参数
        argv = argv[:argv.index('--ros-args')]
    if len(argv) < 2:
        print(__doc__)
        sys.exit(2)
    data_dir, ts_file = argv[0], argv[1]
    freq = float(argv[2]) if len(argv) > 2 else 10.0
    maxf = int(argv[3]) if len(argv) > 3 else int(os.environ.get('MAX_FRAMES', '0'))

    rclpy.init()
    n = KittiPubVis(data_dir, ts_file, freq=freq, max_frames=maxf)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        import time as _t
        _t.sleep(0.5)
        os._exit(0)


if __name__ == '__main__':
    main()
