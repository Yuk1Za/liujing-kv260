#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rec_traj.py —— 记录后端轨迹并统计共存节拍（一次性小工具，不参与算法）。

为什么需要它：`map_optimization` 把位姿发在 `mapping/odometry` 上，但把 `mapping/path`
整个 Path 消息反复发（消息体积大）。这里订 odometry 就够了：
- 每收到一条写一行 `t x y z qx qy qz qw`；
- 退出时打印条数与平均节拍 ⇒ **共存到底跑多快**这个数就有了。

用法（板端）：
    python3 rec_traj.py /home/ubuntu/lvi_board/map_$TAG.txt [duration_s]
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class Rec(Node):
    def __init__(self, path):
        super().__init__('rec_traj')
        self.f = open(path, 'w')
        self.n = 0
        self.t0 = None
        self.t1 = None
        # ★ map_optimization 用 `create_publisher(..., 1)` ⇒ 默认 **RELIABLE**。
        #   这里若订 BEST_EFFORT 会 QoS 不兼容、一条都收不到（现象：文件 0 行且无报错）。
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Odometry, '/mapping/odometry', self.cb, qos)
        self.get_logger().info(f'rec_traj → {path}')

    def cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        self.t1 = t
        self.f.write(f'{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                     f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
        self.n += 1
        if self.n % 10 == 0:
            self.f.flush()

    def close(self):
        self.f.flush()
        self.f.close()
        dur = (self.t1 - self.t0) if (self.t0 and self.t1) else 0.0
        hz = (self.n - 1) / dur if dur > 0 else 0.0
        print(f'[rec_traj] 条数 {self.n}  时间跨度 {dur:.1f} s  ⇒ 节拍 {hz:.2f} Hz')


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/lvi_board/map_traj.txt'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    rclpy.init()
    n = Rec(path)
    try:
        if dur > 0:
            import time
            end = time.time() + dur
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.2)
        else:
            rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.close()
        try:
            n.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
