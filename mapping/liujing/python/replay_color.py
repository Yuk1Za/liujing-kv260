"""Restore camera's native MJPEG recording to the identical BGR ROS input."""
import cv2,numpy as np,rclpy
from array import array
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CompressedImage


def main():
    cv2.setNumThreads(1);rclpy.init();node=Node('liujing_replay_color')
    pub=node.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data)
    def receive(msg):
        frame=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
        if frame is None:raise ValueError('invalid recorded camera JPEG')
        out=Image();out.header=msg.header;out.height,out.width=frame.shape[:2];out.encoding='bgr8';out.step=out.width*3;out.data=array('B',frame.tobytes());pub.publish(out)
    node.create_subscription(CompressedImage,'/camera/color/image_raw/compressed',receive,qos_profile_sensor_data)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
