"""Bounded joint RGB-D/C16 capture; no pose assumption and no hardware loading."""
import argparse,json,time
from collections import deque,Counter
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,PointCloud2
from std_msgs.msg import String
from nav_msgs.msg import Odometry

p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=25);p.add_argument('--sample-period',type=float,default=1);p.add_argument('--record-poses',action='store_true');a=p.parse_args()
if not 1<=a.seconds<=300 or not .5<=a.sample_period<=10:raise ValueError('capture must be bounded to 300 seconds and at most 2 Hz')
out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
class Capture(Node):
 def __init__(self):
  super().__init__('liujing_alignment_capture');self.rgb=deque(maxlen=8);self.depth=deque(maxlen=8);self.count=Counter();self.saved=0;self.last=0;self.metadata=open(out/'metadata.jsonl','w',buffering=1)
  self.create_subscription(Image,'/camera/color/image_raw',lambda m:self.image(m,'rgb'),qos_profile_sensor_data)
  self.create_subscription(Image,'/camera/depth/image_raw',lambda m:self.image(m,'depth'),qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/point_cloud_raw',self.cloud,QoSProfile(depth=2,reliability=ReliabilityPolicy.RELIABLE))
  self.create_subscription(String,'/camera/frame_metadata',lambda m:self.metadata.write(m.data+'\n'),qos_profile_sensor_data)
  self.poses=open(out/'poses.jsonl','w',buffering=1) if a.record_poses else None
  if self.poses:
   for kind,topic in [('raw','/odom_init/laser_odom'),('mapped','/mapping/odometry')]:
    self.create_subscription(Odometry,topic,lambda m,k=kind:self.pose(m,k),QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE))
 def pose(self,m,kind):
  p,q=m.pose.pose.position,m.pose.pose.orientation;self.count[kind+'_poses']+=1
  self.poses.write(json.dumps(dict(kind=kind,source_stamp=stamp(m),host_monotonic=time.monotonic(),frame=m.header.frame_id,child=m.child_frame_id,position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w]))+'\n')
 def image(self,m,kind):
  self.count[kind]+=1;getattr(self,kind).append((stamp(m),time.monotonic(),m))
 def cloud(self,m):
  self.count['lidar']+=1;now=time.monotonic()
  if now-self.last<a.sample_period or not self.rgb or not self.depth:return
  t=stamp(m);c=min(self.rgb,key=lambda x:abs(x[0]-t));d=min(self.depth,key=lambda x:abs(x[0]-t))
  if abs(c[0]-d[0])>.04 or max(abs(c[0]-t),abs(d[0]-t))>.2:self.count['unmatched']+=1;return
  rgb=c[2];dep=d[2]
  if rgb.encoding!='bgr8' or dep.encoding!='16UC1':raise ValueError('unexpected image encoding')
  im=np.frombuffer(rgb.data,np.uint8).reshape(rgb.height,rgb.step)[:,:rgb.width*3].reshape(rgb.height,rgb.width,3)[:,:,::-1].copy()
  depth=np.frombuffer(dep.data,np.dtype(('>' if dep.is_bigendian else '<')+'u2')).reshape(dep.height,dep.step//2)[:,:dep.width].astype(np.float32)*.001
  fields={f.name:f for f in m.fields};endian='>' if m.is_bigendian else '<'
  if any(fields[k].datatype!=7 for k in ('x','y','z')):raise ValueError('expected float XYZ')
  xyz=np.stack([np.ndarray((m.height,m.width),dtype=endian+'f4',buffer=m.data,offset=fields[k].offset,strides=(m.row_step,m.point_step)).ravel() for k in ('x','y','z')],-1);xyz=xyz[np.isfinite(xyz).all(1)&(np.linalg.norm(xyz,axis=1)>.15)]
  np.savez_compressed(out/f'{self.saved:03}.npz',rgb=im,depth=depth,lidar_xyz=xyz,stamps=np.array([c[0],d[0],t]),host_monotonic=np.array([c[1],d[1],now]),lidar_frame=np.array(m.header.frame_id))
  self.saved+=1;self.last=now
rclpy.init();n=Capture();start=time.monotonic()
try:
 while rclpy.ok() and time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.1)
finally:
 report={'scope':'joint observation for calibration checking; host-stamp pairing, not time calibration','saved':n.saved,'counts':dict(n.count),'elapsed_s':time.monotonic()-start};(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));n.metadata.close()
 if n.poses:n.poses.close()
 n.destroy_node();rclpy.shutdown()
