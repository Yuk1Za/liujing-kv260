#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""录 pl_vision 的原始数据，供【离线复算】两种饱和策略用

为什么要录：门控（sat_gate）只是一个逐帧判决，判据全是 flow 里的
(frac_sat, median(u)) —— 把 flow 原样录下来，就能在同一批数据上
同时复算「无门控」与「有门控」两条曲线，不受两次运行节奏不同的干扰。

录三样：
  pl_vision/flow           N×6 [x,y,u,v,status,frame]  （每帧一条，frame 在第 5 列）
  pl_vision/sync           10 列，取 [2]=图像帧号 [3]=雷达帧戳（给 flow 帧号配雷达戳）
  odom_init/laser_odom     header.stamp -> yaw（雷达真值）

用法（板端，root）：
  sudo python3 rec_vision.py out.npz
  Ctrl-C / SIGTERM 收尾时落盘
"""
import math
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from nav_msgs.msg import Odometry


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Rec(Node):
    def __init__(self, out):
        super().__init__('rec_vision')
        self.out = out
        self.flows = []
        self.sync = {}
        self.lidar = {}
        self.n_flow = 0
        self.n_lidar = 0
        self.create_subscription(Float32MultiArray, 'pl_vision/flow', self.on_flow, 50)
        self.create_subscription(Float64MultiArray, 'pl_vision/sync', self.on_sync, 50)
        self.create_subscription(Odometry, 'odom_init/laser_odom', self.on_lidar, 500)
        self.get_logger().info('录制中 -> %s' % out)

    def on_flow(self, msg):
        d = np.asarray(msg.data, np.float32)
        if d.size == 0 or d.size % 6:
            return
        a = d.reshape(-1, 6)
        self.flows.append(a.copy())
        self.n_flow += 1

    def on_sync(self, msg):
        if len(msg.data) < 10:
            return
        self.sync[int(round(float(msg.data[2])))] = float(msg.data[3])

    def on_lidar(self, msg):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        self.lidar[st] = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.n_lidar += 1

    def save(self):
        flow = (np.concatenate(self.flows, 0) if self.flows
                else np.zeros((0, 6), np.float32))
        sync = np.array(sorted(self.sync.items()), np.float64).reshape(-1, 2)
        lidar = np.array(sorted(self.lidar.items()), np.float64).reshape(-1, 2)
        np.savez_compressed(self.out, flow=flow, sync=sync, lidar=lidar)
        print('落盘 %s：flow %d 条/%d 点、sync %d、lidar %d'
              % (self.out, self.n_flow, flow.shape[0], sync.shape[0], lidar.shape[0]))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/pl_vision/logs/rec.npz'
    rclpy.init()
    node = Rec(out)

    def bye(*_a):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, bye)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
