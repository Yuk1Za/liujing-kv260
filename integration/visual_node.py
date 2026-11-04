#!/usr/bin/env python3
"""共享相机图像 + C16 赋深 → PL 特征跟踪 → PnP → 后端视觉位姿。"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"vision/lvi_board"))
from pl_match import PLMatcher
from vo_core import PnPVO
from visual_frontend import VisualPose, VisualPosePublisher

def stamp(msg):
    return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec

class VisualNode(Node):
    def __init__(self,args):
        super().__init__("liujing_visual_frontend")
        cfg=json.loads(Path(args.calibration).read_text())
        c=cfg["color"]
        self.K=np.array([[c["fx"],0,c["cx"]],[0,c["fy"],c["cy"]],[0,0,1.]])
        self.dist=np.asarray(c["distortion"],float)
        self.Tbc=np.asarray(cfg["T_pose_depth"],float)@np.linalg.inv(np.asarray(cfg["T_color_depth"],float))
        self.Tcb=np.linalg.inv(self.Tbc)
        self.matcher=PLMatcher(bit=args.bit,w=c["width"],h=c["height"],nms="on")
        self.vo=PnPVO(self.K)
        self.publisher=VisualPosePublisher(self)
        self.clouds=deque(maxlen=20)
        self.image=None
        self.previous=None
        self.processed=-1
        self.create_subscription(PointCloud2,"/point_cloud_raw",lambda m:self.clouds.append(m),qos_profile_sensor_data)
        self.create_subscription(CompressedImage,"/liujing/camera/color/compressed",self.on_image,qos_profile_sensor_data)
        self.create_timer(.1,self.process)

    def on_image(self,msg):
        self.image=msg

    def project(self,cloud):
        xyz=read_points_numpy(cloud,field_names=("x","y","z"),skip_nans=True).reshape(-1,3)
        camera=xyz@self.Tcb[:3,:3].T+self.Tcb[:3,3]
        camera=camera[np.isfinite(camera).all(axis=1)&(camera[:,2]>.1)]
        pix=camera@self.K.T
        return pix[:,:2]/pix[:,2,None],camera[:,2]

    def process(self):
        msg=self.image
        if msg is None or not self.clouds or stamp(msg)<=self.processed:return
        cloud=min(self.clouds,key=lambda c:abs(stamp(c)-stamp(msg)))
        if abs(stamp(cloud)-stamp(msg))>150000000:return
        gray=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
        if gray is None:return
        gray=cv2.undistort(gray,self.K,self.dist)
        self.processed=stamp(msg)
        if self.previous is None:
            self.previous=(gray,cloud)
            self.publisher.publish(VisualPose(stamp(msg),np.eye(4),0,0.))
            return
        matched=self.matcher.match(self.previous[0],gray,det="st")
        uv,z=self.project(self.previous[1])
        result=self.vo.step(matched["uv_prev"],matched["uv_cur"],uv,z)
        if result is None:return
        pose=self.Tbc@self.vo.pose()@self.Tcb
        self.publisher.publish(VisualPose(stamp(msg),pose,result["n_inliers"],result["rms_px"]))
        self.previous=(gray,cloud)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--bit",required=True)
    p.add_argument("--calibration",required=True)
    args,ros=p.parse_known_args()
    os.environ["PL_DOWNLOAD"]="0"
    os.environ["PL_CLEAR_CACHE"]="0"
    rclpy.init(args=ros)
    node=VisualNode(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
