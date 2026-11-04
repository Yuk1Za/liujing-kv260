#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KITTI 0027 回放 publisher：发 /velodyne_points(PointCloud2) + /camera/image_raw(Image)。

为什么自己写
------------
板上没有现成的点云 publisher（lo_sam 源码在服务器，不在板上）。
为了端到端验证视觉节点，一个 60 行的 publisher 比去复原别人的 launch 更可控。

图像在**发之前**就 letterbox 到 640×480（与板上 PL 前端一致），
这样 vo 节点不必再做 resize，也避免每帧多传 1.36 MB 原图。
"""
import argparse
import os
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header


def make_pc2(xyz_i, stamp, frame='velodyne'):
    """xyz_i: (N,4) float32。按 x,y,z,intensity 打包成 PointCloud2。"""
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame)
    msg.height = 1
    msg.width = int(len(xyz_i))
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * msg.width
    msg.is_dense = True
    msg.data = np.ascontiguousarray(xyz_i, dtype=np.float32).tobytes()
    return msg


class KittiPub(Node):
    def __init__(self, a):
        super().__init__('kitti_pub')
        self.a = a
        # ★ 960 KB 的点云消息会被分片成几百个 UDP 包；RELIABLE 要逐包 ACK，
        #   板上实测把节拍拖到 ~1 帧/s。点云是"最新帧有用"的流数据 ⇒ BEST_EFFORT + depth=1。
        qos = QoSProfile(depth=1)
        # ★ QoS 必须与**订阅方**匹配，否则一条都发不出去（只在 publisher 侧留一行 WARN，
        #   订阅侧完全静默，极难查）。实测：
        #   · 我们的 vo_pnp_node 订 BEST_EFFORT ⇒ 单跑一律 BEST_EFFORT；
        #   · lo_sam 的 feature_extract 订 **RELIABLE** ⇒ 共存必须切 reliable，
        #     否则报 "requesting incompatible QoS. No messages will be sent to it"。
        qos.reliability = (ReliabilityPolicy.RELIABLE if a.pc_qos == 'reliable'
                           else ReliabilityPolicy.BEST_EFFORT)
        self.p_pc = self.create_publisher(PointCloud2, a.pc_topic, qos)
        # 视觉专用抽稀点云**恒用 BEST_EFFORT**：视觉节点就是 BEST_EFFORT 订阅；
        # 抽稀的意义就是别再走可靠重传（976 KB 分片逐包 ACK 会把节拍拖到 ~1 Hz）。
        qvis = QoSProfile(depth=1)
        qvis.reliability = ReliabilityPolicy.BEST_EFFORT
        self.p_vis_pc = self.create_publisher(PointCloud2, a.vis_topic, qvis)
        self.p_img = self.create_publisher(Image, a.img_topic, qvis)
        self.i = a.F0
        self.end = a.F0 + a.W
        self.per = 1.0 / a.rate
        self.timer = self.create_timer(self.per, self.tick)
        self.get_logger().info(
            f'回放 {a.F0}..{self.end-1} @ {a.rate} Hz  图 {a.wimg}x{a.himg}  '
            f'pc→{a.pc_topic}  img→{a.img_topic}')

    def tick(self):
        if self.i >= self.end:
            self.get_logger().info(f'回放结束（{self.end - self.a.F0} 帧），关闭 publisher')
            # ★ 不要在 timer 回调里 raise SystemExit：DDS 传输队列有积压时进程会挂住，
            #   实测 200 帧那次 publisher 一直不退出，把上层脚本拖到 timeout。
            #   正确做法是取消定时器 + shutdown()，让 spin() 自然返回。
            self.timer.cancel()
            # ★ 不要走 rclpy.shutdown()：板上实测卡在 DDS 拆除、忽略 SIGTERM
            import time as _t
            _t.sleep(1.0)      # 让最后几条消息真发出去
            os._exit(0)        # 硬退出，避免收尾挂死
            return
        idx = self.i
        png = f'{self.a.img_dir}/{idx:010d}.png'
        binf = f'{self.a.vel_dir}/{idx:010d}.bin'
        if not (os.path.exists(png) and os.path.exists(binf)):
            self.get_logger().warn(f'缺数据，跳过 f{idx}')
            self.i += 1
            return
        stamp = self.get_clock().now().to_msg()

        xyz_i = np.fromfile(binf, dtype=np.float32).reshape(-1, 4)
        if self.a.stride > 1:
            xyz_i = xyz_i[::self.a.stride]

        # ★ 默认**不发图像**：板上实测 ~1 MB 图像消息 + 960 KB 点云走 DDS，
        #   会把回放节拍拖到 ~2.2 s/帧。视觉节点改为本地读图（与 pl_vision_node 一致）。
        if self.a.send_img:
            im = cv2.imread(png)
            nh = int(round(im.shape[0] * self.a.wimg / im.shape[1]))
            r = cv2.resize(im, (self.a.wimg, nh), interpolation=cv2.INTER_AREA)
            out = np.zeros((self.a.himg, self.a.wimg, 3), np.uint8)
            top = (self.a.himg - nh) // 2
            out[top:top + nh] = r
            imsg = Image()
            imsg.header = Header(stamp=stamp, frame_id='camera')
            imsg.height, imsg.width = self.a.himg, self.a.wimg
            imsg.encoding = 'bgr8'
            imsg.is_bigendian = 0
            imsg.step = self.a.wimg * 3
            imsg.data = out.tobytes()
            self.p_img.publish(imsg)
        # ★ 帧号写进 frame_id：订阅端不能用「收到第几条」反推帧号——
        #   BEST_EFFORT 会丢消息，一丢就整条链错位（实测误差 35–46 m）。
        self.p_pc.publish(make_pc2(xyz_i, stamp, frame=f'velodyne:{idx}'))

        self.i += 1

        if self.a.vis_stride > 1:
            _v = xyz_i[::self.a.vis_stride]
            self.p_vis_pc.publish(make_pc2(_v, stamp, frame=f'velodyne:{idx}'))

def main():
    ap = argparse.ArgumentParser()
    H = os.path.expanduser('~')
    ap.add_argument('--F0', type=int, default=0)
    ap.add_argument('--W', type=int, default=200)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--stride', type=int, default=2)
    ap.add_argument('--vis_stride', type=int, default=4,
                    help='给视觉节点的抽稀倍数（1=关；点云 DDS ~1.2 s/帧，抽稀是唯一杠杆）')
    ap.add_argument('--vis_topic', default='/velodyne_points_vis')
    ap.add_argument('--pc_qos', choices=('best_effort', 'reliable'), default='best_effort',
                    help='主点云 QoS：自己节点用 best_effort；与 lo_sam 共存必须 reliable')
    ap.add_argument('--wimg', type=int, default=640)
    ap.add_argument('--himg', type=int, default=480)
    ap.add_argument('--img_dir', default=f'{H}/kitti_cam/image_02/data')
    ap.add_argument('--vel_dir', default=f'{H}/_kitti_incoming/velodyne_points/data')
    ap.add_argument('--pc_topic', default='/velodyne_points')
    ap.add_argument('--img_topic', default='/camera/image_raw')
    ap.add_argument('--send_img', action='store_true',
                    help='同时发图像话题（默认不发；DDS 传 1 MB 消息在板上很慢）')
    a = ap.parse_args()

    rclpy.init()
    n = KittiPub(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
