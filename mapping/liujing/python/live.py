"""ROS 2 acquisition/CPU baseline. Requires accepted calibration, never chassis."""
import argparse,json,time,threading,math,signal,os
from array import array
from pathlib import Path
from collections import deque,Counter
import numpy as np
from native import Camera,Map,RejectedFrame
from alignment import PairQueue,PoseBuffer,checked_pose,quat_matrix,validate_calibration,register_color
from viewer import FrameServer
from online_contract import validate_session,StaticGuard,pose_support,finite_rate

def matrix_quat(R):
    # Symmetric eigenproblem handles 180-degree rotations without division by zero.
    K=np.array([[R[0,0]-R[1,1]-R[2,2],R[0,1]+R[1,0],R[0,2]+R[2,0],R[2,1]-R[1,2]],
     [R[0,1]+R[1,0],R[1,1]-R[0,0]-R[2,2],R[1,2]+R[2,1],R[0,2]-R[2,0]],
     [R[0,2]+R[2,0],R[1,2]+R[2,1],R[2,2]-R[0,0]-R[1,1],R[1,0]-R[0,1]],
     [R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1],np.trace(R)]])/3
    _,v=np.linalg.eigh(K);q=v[:,-1];return q if q[3]>=0 else -q

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--calibration',required=True);ap.add_argument('--out',required=True);ap.add_argument('--host',default='127.0.0.1');ap.add_argument('--port',type=int,default=8090);ap.add_argument('--pose-topic',default='/odom_init/laser_odom');ap.add_argument('--seconds',type=float,default=0);ap.add_argument('--backend',choices=['cpu','pl'],default='cpu');ap.add_argument('--numeric',choices=['fp32','mixed','qrcp_v1'],default='fp32');ap.add_argument('--bit')
    ap.add_argument('--session-mode',choices=['calibrated','static_diagnostic'],default='calibrated');ap.add_argument('--fusion-hz',type=finite_rate,default=1);ap.add_argument('--render-hz',type=finite_rate,default=2);ap.add_argument('--pose-wait-seconds',type=finite_rate,default=.5);ap.add_argument('--static-warmup-seconds',type=float,default=10);ap.add_argument('--observe-static-drift',action='store_true');ap.add_argument('--use-sim-time',action='store_true');a=ap.parse_args()
    if a.observe_static_drift and a.session_mode!='static_diagnostic':ap.error('drift observation is only for operator-confirmed stationary diagnostics')
    if not 0<=a.static_warmup_seconds<=30:ap.error('static warmup must be between zero and 30 seconds')
    if a.pose_wait_seconds>2:ap.error('pose wait must be bounded to at most two seconds')
    if a.backend=='pl' and (not a.bit or a.numeric not in ('mixed','qrcp_v1')):ap.error('PL stream backend requires --bit and an explicit supported numeric contract')
    cal=json.loads(Path(a.calibration).read_text());validate_session(cal,a.session_mode)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    root=Path(a.out);root.mkdir(parents=True,exist_ok=False);(root/'calibration.json').write_text(json.dumps(cal,indent=2));cam=Camera.from_dict(cal['depth']);viewcam=Camera(640,480,cam.fx,cam.fy,319.5,239.5,cam.near_z,cam.far_z);(root/'camera.json').write_text(json.dumps(cam.as_dict()))
    def ts(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    class Live(Node):
      def __init__(self):
        super().__init__('liujing',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=a.use_sim_time)]);self.map=Map();self.lock=threading.Lock();self.pairs=PairQueue();self.poses=PoseBuffer();self.pending=deque();self.stats=Counter();self.stop=threading.Event();self.view={'mode':'follow','render_mode':0};self.latest=np.eye(4);self.free=np.eye(4);self.path=[];self.last_frame=-1;self.last_render_version=-1;self.last_view=None;self.latest_stamp=None;self.failure=None;self.log=open(root/'events.jsonl','w',buffering=1)
        self.static_guard=StaticGuard() if a.session_mode=='static_diagnostic' else None
        self.first_pose_stamp=None;self.pose_initializing=bool(self.static_guard and a.static_warmup_seconds)
        self.static_drift_exceeded=False
        self.last_render_mono=0;self.pose_sources=None;self.latest_rgb_stamp=None;self.input_received=None;self.completed_pl_jobs=0;self.last_input_mono={};self.started_mono=time.monotonic();self.view_request=0;self.rendered_request=0;self.last_selected_stamp=-1
        self.raster=None;self.renderer_lease=None;self.association_ms=0;self.pl_stages={}
        if a.backend=='pl':
          # Attach only. Reconfiguring here would destroy concurrent C16 state.
          import hashlib
          from pynq import Overlay,PL
          from accelerator import Raster
          from hardware_ownership import claim_renderer
          self.renderer_lease=claim_renderer()
          current=Path(PL.bitfile_name or '')
          if not current.is_file() or hashlib.sha256(current.read_bytes()).digest()!=hashlib.sha256(Path(a.bit).read_bytes()).digest():raise RuntimeError('load the selected integrated overlay before starting localization and LiuJing')
          self.raster=Raster(Overlay(a.bit,download=False),surfel_capacity=1000000,numeric_contract=a.numeric)
          if self.raster.mode_offset is None:raise RuntimeError('overlay lacks association-only mode')
        self.poses.max_speed=cal.get('max_pose_speed_mps',2.0);self.poses.max_angular_speed=cal.get('max_pose_angular_speed_radps',2.0)
        self.pub_image=self.create_publisher(Image,'/liujing/render/image',qos_profile_sensor_data);self.pub_pose=self.create_publisher(PoseStamped,'/liujing/pose',10);self.pub_state=self.create_publisher(String,'/liujing/status',10);self.pub_pose_state=self.create_publisher(String,'/liujing/pose_status',10)
        self.create_subscription(Image,'/camera/color/image_raw',lambda m:self.image(m,'rgb'),qos_profile_sensor_data);self.create_subscription(Image,'/camera/depth/image_raw',lambda m:self.image(m,'depth'),qos_profile_sensor_data)
        self.create_subscription(Odometry,a.pose_topic,self.pose,QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE));self.create_subscription(PoseStamped,'/liujing/view_pose',self.external_view,10)
        self.server=FrameServer(a.host,a.port,self.control);self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
        if os.environ.get('LIUJING_READY_DIR'):Path(os.environ['LIUJING_READY_DIR'],'visual.ready').write_text('attached\n')
      def image(self,m,kind):
        try:
          if kind=='rgb':
            if m.encoding not in ('rgb8','bgr8'):raise ValueError('color encoding')
            im=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
            if m.encoding=='bgr8':im=im[:,:,::-1].copy()
          else:
            if m.encoding not in ('16UC1','32FC1'):raise ValueError('depth encoding')
            dtype=np.dtype(('>' if m.is_bigendian else '<')+('u2' if m.encoding=='16UC1' else 'f4'));im=np.frombuffer(m.data,dtype).reshape(m.height,m.step//dtype.itemsize)[:,:m.width].astype(np.float32)
            if m.encoding=='16UC1':im*=.001
          stamp=ts(m)+cal.get(('rgb' if kind=='rgb' else 'depth')+'_time_offset_s',0)
          with self.lock:
            self.last_input_mono[kind]=time.monotonic();self.pairs.push(kind,stamp,im,self.last_input_mono[kind])
        except (ValueError,TypeError):
          with self.lock:self.stats['invalid_'+kind]+=1
      def pose(self,m):
        try:
          if m.header.frame_id!=cal.get('source_frame','camera_init') or m.child_frame_id!=cal.get('source_child_frame','laser_odom'):raise ValueError('pose frame mismatch')
          p=m.pose.pose;q=p.orientation;T=np.eye(4);T[:3,:3]=quat_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[p.position.x,p.position.y,p.position.z]
          T=np.asarray(cal['T_map_source'])@T@np.asarray(cal['T_pose_depth']);checked_pose(T)
          with self.lock:
            if self.first_pose_stamp is None:self.first_pose_stamp=ts(m)
            if self.static_guard and ts(m)-self.first_pose_stamp<a.static_warmup_seconds:
              self.stats['pose_initialization_skipped']+=1;self.last_input_mono['pose']=time.monotonic();return
            self.pose_initializing=False
            if self.static_guard:
              try:self.static_guard.check(T)
              except ValueError:
                if not a.observe_static_drift:raise
                self.static_drift_exceeded=True;self.stats['static_drift_limit_exceeded']+=1
            self.last_input_mono['pose']=time.monotonic();accepted=self.poses.add(ts(m),T)
            if self.poses.fault:self.failure=self.poses.fault;self.stop.set()
          state=String();state.data=json.dumps({'accepted':accepted,'source_stamp':ts(m),'source':'lidar_only','fault':self.poses.fault,'check_scope':'rigidity, chronology, frame and motion plausibility; not estimator convergence','extrinsics':a.calibration});self.pub_pose_state.publish(state)
        except ValueError as exc:
          with self.lock:
            self.stats['invalid_pose']+=1
            if 'static diagnostic motion' in str(exc):self.failure=str(exc);self.stop.set()
      def external_view(self,m):
        if m.header.frame_id!='map':return
        try:
          p=m.pose;q=p.orientation;T=np.eye(4);T[:3,:3]=quat_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[p.position.x,p.position.y,p.position.z];checked_pose(T)
          with self.lock:
            self.free=T;self.view['mode']='free';self.view_request+=1
        except ValueError:pass
      def control(self,v):
        with self.lock:
          if self.view_request>self.rendered_request:self.stats['view_superseded']+=1
          self.view_request+=1
          if 'mode' in v:
            if v['mode'] not in ('follow','top','free'):raise ValueError('view mode')
            if self.view['mode']!='free' and v['mode']=='free':self.free=self.latest.copy()
            self.view['mode']=v['mode']
          if 'render_mode' in v:
            if v['render_mode'] not in (0,1,2):raise ValueError('render mode')
            self.view['render_mode']=v['render_mode']
          for key,axis in [('dx',0),('dz',2)]:
            if key in v:
              step=float(v[key])
              if not math.isfinite(step) or abs(step)>.5:raise ValueError('view step')
              self.free[:3,3]+=self.free[:3,axis]*step;self.view['mode']='free'
          if 'yaw' in v:
            angle=float(v['yaw'])
            if not math.isfinite(angle) or abs(angle)>.5:raise ValueError('yaw')
            if self.view['mode']!='free':self.free=self.latest.copy()
            c,s=math.cos(angle),math.sin(angle);self.free[:3,:3]=self.free[:3,:3]@np.array([[c,0,s],[0,1,0],[-s,0,c]]);self.view['mode']='free'
      def run(self):
        while not self.stop.is_set():
          now=time.monotonic();chosen=None
          with self.lock:
            while True:
              pair=self.pairs.take(now)
              if pair is None:break
              if pair[1][0]-self.last_selected_stamp<1/a.fusion_hz-1e-6:self.stats['rate_selected_out']+=1;continue
              self.last_selected_stamp=pair[1][0]
              if len(self.pending)>=8:self.pending.popleft();self.stats['pose_wait_overflow']+=1
              self.pending.append(pair)
            while self.pending:
              pair=self.pending[0];stamp=pair[1][0]
              if now-pair[1][2]>a.pose_wait_seconds:self.pending.popleft();self.stats['pose_wait_expired']+=1;continue
              T=self.poses.at(stamp)
              if T is None:break
              self.pending.popleft()
              if stamp-self.last_frame<1/a.fusion_hz-1e-6:self.stats['rate_selected_out']+=1;continue
              chosen=(pair,T,pose_support(self.poses.items,stamp,self.poses.max_gap));break
          try:
            fusion_ms=0
            if chosen:
              (rgbitem,depitem),T,self.pose_sources=chosen;stamp=depitem[0];self.latest_rgb_stamp=rgbitem[0];self.input_received=depitem[2];begin=time.perf_counter();d,rgb=register_color(depitem[1],rgbitem[1],cal,native_library=self.map.lib)
              association=None;self.association_ms=0
              if self.raster:
                version=self.map.version;astart=time.perf_counter();_,rd,ri=self.raster.render(self.map,cam,T,3,frame_id=stamp);self.completed_pl_jobs+=1;self.association_ms=(time.perf_counter()-astart)*1000;association=(rd,ri,version);self.pl_stages={'association':dict(self.raster.timing),'association_job':dict(self.raster.last_job)}
              elif a.numeric in ('mixed','qrcp_v1'):
                _,rd,ri=self.map.render(cam,T,3);association=(rd,ri,self.map.version)
              added=self.map.integrate(d,rgb,cam,T,stamp,association=association);fusion_ms=(time.perf_counter()-begin)*1000;self.last_frame=stamp;self.latest_stamp=stamp
              with self.lock:self.latest=T;self.path.append(T[:3,3].copy());self.path=self.path[-5000:];self.stats['fused']+=1
              pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp.sec=int(stamp);pose.header.stamp.nanosec=int((stamp-int(stamp))*1e9);pose.pose.position.x=float(T[0,3]);pose.pose.position.y=float(T[1,3]);pose.pose.position.z=float(T[2,3]);q=matrix_quat(T[:3,:3]);pose.pose.orientation.x=float(q[0]);pose.pose.orientation.y=float(q[1]);pose.pose.orientation.z=float(q[2]);pose.pose.orientation.w=float(q[3]);self.pub_pose.publish(pose)
            with self.lock:
              view=dict(self.view);request=self.view_request;T=self.latest.copy() if view['mode']=='follow' else self.free.copy()
              if view['mode']=='top':
                # map axes are explicitly verified to ROS x/y horizontal, z up.
                T=np.eye(4);T[:3,:3]=np.diag([1,-1,-1]);T[:3,3]=self.latest[:3,3]+[0,0,3]
              stats=dict(self.stats+self.pairs.stats+self.poses.stats);input_times=dict(self.last_input_mono)
            render_ms=0;compose_ms=0;ros_image_publish_ms=0
            if self.map.count and now-self.last_render_mono>=1/a.render_hz and (self.last_render_version!=self.map.version or self.last_view is None or not np.array_equal(T,self.last_view[0]) or view['render_mode']!=self.last_view[1]):
              start=time.perf_counter()
              if self.raster:
                im,dep,ids=self.raster.render(self.map,viewcam,T,view['render_mode'],frame_id=self.latest_stamp);self.completed_pl_jobs+=1;self.pl_stages['display']=dict(self.raster.timing);self.pl_stages['display_job']=dict(self.raster.last_job)
              else:im,dep,ids=self.map.render(viewcam,T,{'mixed':4,'qrcp_v1':5}.get(a.numeric,0) if view['render_mode']==0 else view['render_mode'])
              render_ms=(time.perf_counter()-start)*1000;self.last_render_version=self.map.version;self.last_view=(T.copy(),view['render_mode']);self.last_render_mono=time.monotonic();self.rendered_request=request;self.stats['rendered']+=1
              from viewer import compose_trajectory
              compose_start=time.perf_counter()
              label=('STATIC DIAGNOSTIC - POSE DRIFT / UNVERIFIED' if self.static_drift_exceeded else 'STATIC DIAGNOSTIC - UNVERIFIED CALIBRATION') if self.static_guard else None
              canvas=compose_trajectory(im,self.path,T,viewcam,label=label)
              im=np.asarray(canvas);compose_ms=(time.perf_counter()-compose_start)*1000;publish_start=time.perf_counter()
              msg=Image();msg.header.frame_id='liujing_virtual_camera';msg.header.stamp=self.get_clock().now().to_msg();msg.height=viewcam.height;msg.width=viewcam.width;msg.encoding='rgb8';msg.step=viewcam.width*3;msg.data=array('B',im.tobytes());self.pub_image.publish(msg);self.current_image=im
              ros_image_publish_ms=(time.perf_counter()-publish_start)*1000
            state={'backend':a.backend,'numeric':a.numeric,'mode':a.session_mode,'calibration_verified':cal.get('verified',{}),'map_version':self.map.version,'map_source_stamp':self.latest_stamp,'rgb_source_stamp':self.latest_rgb_stamp,'pose_source_stamps':self.pose_sources,'map_age_s':None if self.latest_stamp is None else self.get_clock().now().nanoseconds/1e9-self.latest_stamp,'surfels':self.map.count,'capacity_drops':self.map.capacity_drops,'fusion_ms':fusion_ms,'association_ms':self.association_ms,'render_ms':render_ms,'pl_stages':self.pl_stages,'counts':stats,'failure':self.failure,'pl_completed_jobs':self.completed_pl_jobs,'fusion_hz_limit':a.fusion_hz,'render_hz_limit':a.render_hz,'input_silence_s':{k:time.monotonic()-v for k,v in input_times.items()}}
            state['input_health']='stale_or_missing' if any(time.monotonic()-input_times.get(k,self.started_mono)>5 for k in ('rgb','depth','pose')) else 'receiving'
            state['map_additions_paused']=self.map.count>=1000000;state['pose_fault']=self.poses.fault
            state['pose_topic']=a.pose_topic;state['pose_wait_limit_s']=a.pose_wait_seconds
            state['compose_ms']=compose_ms;state['ros_image_publish_ms']=ros_image_publish_ms
            state['pose_initializing']=self.pose_initializing;state['static_warmup_seconds']=a.static_warmup_seconds if self.static_guard else 0
            state['static_drift_exceeded']=self.static_drift_exceeded;state['static_drift_policy']='observe_unverified' if a.observe_static_drift else 'stop'
            state['fusion_health']='initializing_pose' if self.pose_initializing else ('waiting_for_valid_pose_pair' if self.latest_stamp is None or state['map_age_s']>5 else 'fresh_map')
            with self.lock:state['counts']=dict(self.stats+self.pairs.stats+self.poses.stats)
            state['pl_job']=self.raster.last_job if self.raster else None
            if render_ms:
              encode_start=time.perf_counter();self.server.update(self.current_image,state);state['jpeg_ms']=(time.perf_counter()-encode_start)*1000;state['receive_to_image_ready_ms']=None if self.input_received is None else (time.monotonic()-self.input_received)*1000;self.server.update_status(state)
            else:self.server.update_status(state)
            temp=root/'status.json.tmp';temp.write_text(json.dumps(state));temp.replace(root/'status.json')
            message=String();message.data=json.dumps(state);self.pub_state.publish(message)
            if chosen or render_ms:self.log.write(json.dumps(state)+'\n')
          except RejectedFrame as exc:
            with self.lock:self.stats['fusion_rejected_'+str(exc.code)]+=1
            self.log.write(json.dumps({'dropped_frame':str(exc)})+'\n')
            if exc.code==-3:self.failure='map index capacity exceeded';self.stop.set()
          except Exception as exc:self.failure=repr(exc);self.log.write(json.dumps({'error':self.failure})+'\n');self.server.update_status({'failure':self.failure,'mode':a.session_mode});self.stop.set()
          self.stop.wait(.05)
      def finish(self):
        self.stop.set();self.worker.join()
        self.server.close();(root/'completion.json').write_text(json.dumps({'failure':self.failure,'counts':dict(self.stats+self.pairs.stats+self.poses.stats),'map_version':self.map.version,'surfels':self.map.count,'pl_completed_jobs':self.completed_pl_jobs,'mode':a.session_mode,'static_drift_exceeded':self.static_drift_exceeded,'static_drift_policy':'observe_unverified' if a.observe_static_drift else 'stop','quality_acceptance':'not_passed' if self.static_guard else 'requires_external_review'},indent=2));self.map.close();self.log.close()
        if self.raster:self.raster.close()
        if self.renderer_lease:self.renderer_lease.close()
    rclpy.init();node=Live();start=time.monotonic()
    signal.signal(signal.SIGINT,lambda *_:node.stop.set());signal.signal(signal.SIGTERM,lambda *_:node.stop.set())
    try:
      while rclpy.ok() and not node.stop.is_set() and (a.seconds<=0 or time.monotonic()-start<a.seconds):rclpy.spin_once(node,timeout_sec=.1)
    finally:node.finish();node.destroy_node();rclpy.shutdown()
    if node.failure:raise RuntimeError(node.failure)
if __name__=='__main__':main()
