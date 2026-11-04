"""Compose historical extrinsics as unverified candidates, never enable live fusion."""
import argparse,ast,json,re
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('snapshot');p.add_argument('--factory',required=True);p.add_argument('--out',required=True);a=p.parse_args()
src=Path(a.snapshot);out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
camtext=(src/'camchain.yaml').read_text()
ci=np.array([ast.literal_eval(x) for x in re.findall(r'^  - (\[.*\])$',camtext,re.M)])
if ci.shape!=(4,4):raise ValueError('camera transform parse')
text=(src/'lidar_imu_results.txt').read_text()
blocks=text.split('Homogeneous Transformation Matrix from LiDAR to IMU:')[1:]
li=[np.array([[float(x) for x in line.split()] for line in block.strip().splitlines()[:4]]) for block in blocks]
if len(li)!=2 or any(x.shape!=(4,4) for x in li):raise ValueError('lidar transform parse')
factory=json.loads(Path(a.factory).read_text());cd=np.eye(4);cd[:3,:3]=np.array(factory['rotation_raw']).reshape(3,3);cd[:3,3]=np.array(factory['translation_raw'])*.001
report={'status':'CANDIDATES_ONLY','camera_serial_matches':'AY2M5430013' in (src/'camera_profiles.txt').read_text(),'camera_imu_reprojection_mean_px':.22644881648764473,'camera_imu_time_shift_s':-.020487974424352694,'lidar_imu_reported_time_lag_s':-.289501,'time_offsets_ported_to_new_driver':False,'formula':'T_lidar_depth = inverse(T_imu_lidar) @ inverse(T_color_imu) @ T_color_depth','deployed_jetson_uses':'initialization, not refinement','candidates':{}}
for name,il in zip(('initialization','refinement'),li):
 ld=np.linalg.inv(il)@np.linalg.inv(ci)@cd
 cal={'description':'UNVERIFIED historical Jetson '+name+'; installation/time/held-out alignment not accepted','verified':{'intrinsics':True,'depth_color':False,'pose_camera':False,'time_alignment':False,'pose_axes':False},'depth':factory['depth'],'color':factory['color'],'T_color_depth':cd.tolist(),'T_pose_depth':ld.tolist(),'T_map_source':np.eye(4).tolist(),'rgb_time_offset_s':0.,'depth_time_offset_s':0.,'timestamp_basis':'new host-observation timestamps; historical time offsets intentionally not transferred','provenance':str(src/'manifest.json')}
 (out/(name+'.json')).write_text(json.dumps(cal,indent=2))
 report['candidates'][name]={'T_lidar_depth':ld.tolist(),'lidar_imu_baseline_m':float(np.linalg.norm(il[:3,3])),'lidar_depth_baseline_m':float(np.linalg.norm(ld[:3,3]))}
report['initialization_refinement_translation_difference_m']=float(np.linalg.norm(li[0][:3,3]-li[1][:3,3]))
report['initialization_refinement_rotation_difference_deg']=float(np.degrees(np.arccos(np.clip((np.trace(li[0][:3,:3].T@li[1][:3,:3])-1)/2,-1,1))))
(out/'audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
