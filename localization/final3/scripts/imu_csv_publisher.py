#!/usr/bin/env python3
"""imu_csv_publisher.py —— 把 IMU CSV 按固定频率重发为 sensor_msgs/Imu（时间戳用 CSV 原值）"""
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time


class ImuCsvPublisher(Node):
    def __init__(self, csv_path, topic, hz):
        super().__init__('imu_csv_publisher')
        self.pub = self.create_publisher(Imu, topic, 200)
        with open(csv_path) as f:
            self.rows = [(float(r['t']),
                          float(r['wx']), float(r['wy']), float(r['wz']),
                          float(r['ax']), float(r['ay']), float(r['az']))
                         for r in csv.DictReader(f)]
        self.i = 0
        self.timer = self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(f'imu_csv_publisher: {len(self.rows)} rows @ {hz}Hz -> {topic}')

    def tick(self):
        if self.i >= len(self.rows):
            rclpy.shutdown()
            return
        t, wx, wy, wz, ax, ay, az = self.rows[self.i]
        msg = Imu()
        sec = int(t)
        msg.header.stamp = Time(sec=sec, nanosec=int((t - sec) * 1e9))
        msg.header.frame_id = 'imu'
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = wx, wy, wz
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
        for k in ('orientation.x', 'orientation.y', 'orientation.z'):
            pass
        msg.orientation_covariance[0] = -1.0   # 无姿态输出标记
        self.pub.publish(msg)
        self.i += 1


def main(args=None):
    import sys
    rclpy.init(args=None)
    csv_path = sys.argv[1] if len(sys.argv) > 1 else ''
    topic = sys.argv[2] if len(sys.argv) > 2 else '/livox/imu'
    hz = float(sys.argv[3]) if len(sys.argv) > 3 else 200.0
    node = ImuCsvPublisher(csv_path, topic, hz)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
