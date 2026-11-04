"""Read-only snapshot of the user's existing calibration evidence via SSH/SFTP."""
import argparse,hashlib,json
from pathlib import Path
import paramiko

p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args()
out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
files={
 'camchain.yaml':'/home/jetson/calib/imu_cam_calib_20260928-camchain-imucam.yaml',
 'imucam_results.txt':'/home/jetson/calib/imu_cam_calib_20260928-results-imucam.txt',
 'lidar_imu_results.txt':'/home/jetson/catkin_lidar_calib/src/lidar_imu_init/result/Initialization_result.txt',
 'lidar_imu_config.yaml':'/home/jetson/catkin_lidar_calib/src/lidar_imu_init/config/c16.yaml',
 'deployed_lidar.yaml':'/home/jetson/catkin_coco_live/src/Coco-LIC/config/live/lidar.yaml',
 'deployed_camera.yaml':'/home/jetson/catkin_coco_live/src/Coco-LIC/config/live/camera.yaml',
 'camera_intrinsics.json':'/home/jetson/catkin_camera/artifacts/intrinsics.json',
 'factory_extrinsics.yaml':'/home/jetson/catkin_camera/artifacts/rgb_depth_extrinsics_sdk.yaml',
 'camera_tf.yaml':'/home/jetson/catkin_camera/artifacts/camera_tf.yaml',
 'camera_profiles.txt':'/home/jetson/catkin_camera/artifacts/device_profiles.txt',
 'camera_driver.cpp':'/home/jetson/catkin_camera/src/OrbbecSDK_ROS1-master/src/ob_camera_node.cpp',
}
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('10.201.1.157',port=16666,username='jetson',key_filename=str(Path.home()/'.ssh/jetson_157_ed25519'),look_for_keys=False,allow_agent=False,timeout=10)
manifest=[]
try:
 with c.open_sftp() as s:
  for name,remote in files.items():
   with s.open(remote,'rb') as f:data=f.read()
   (out/name).write_bytes(data)
   manifest.append({'file':name,'source':remote,'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data),'remote_mtime_unverified':s.stat(remote).st_mtime})
finally:c.close()
(out/'manifest.json').write_text(json.dumps({'source_host':'jetson@10.201.1.157:16666','client_date':'2026-10-24','files':manifest},indent=2))
print('Retrieved',len(manifest),'files; no remote modifications')
