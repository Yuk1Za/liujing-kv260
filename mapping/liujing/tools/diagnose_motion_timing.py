"""Offline motion/offset diagnostic; never installs offsets or verifies flags.

Uses capture_motion_calibration.sh observations and actual raw odometry samples.
Camera helper stamps are first host observations, so even an observable optimum
must be checked against the native device-clock mapper before live deployment.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from diagnose_extrinsics import load, correspondences
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'python'))
from alignment import PoseBuffer, quat_matrix, checked_pose


def relative_camera_transform(pose_depth, camera_pose, lidar_pose):
    return np.linalg.inv(pose_depth) @ np.linalg.inv(camera_pose) @ lidar_pose


def main():
    ap=argparse.ArgumentParser();ap.add_argument('capture');ap.add_argument('--calibration',required=True);ap.add_argument('--out',required=True)
    a=ap.parse_args();root=Path(a.capture);out=Path(a.out);out.mkdir(exist_ok=False)
    cal=json.loads(Path(a.calibration).read_text());extrinsic=checked_pose(cal['T_pose_depth'])
    records=[json.loads(s) for s in (root/'poses.jsonl').read_text().splitlines() if s.strip()]
    records=[r for r in records if r['kind']=='raw']
    poses=PoseBuffer(capacity=max(1024,len(records)+1),max_gap=.3)
    matrices=[]
    for r in records:
        T=np.eye(4);T[:3,:3]=quat_matrix(r['quaternion_xyzw']);T[:3,3]=r['position']
        if not poses.add(r['source_stamp'],T):raise ValueError('pose timestamps are not strictly increasing')
        matrices.append(T)
    if len(matrices)<10:raise ValueError('insufficient actual odometry samples')
    displacement=max(float(np.linalg.norm(T[:3,3]-matrices[0][:3,3])) for T in matrices)
    rotation=max(float(np.degrees(np.arccos(np.clip((np.trace(matrices[0][:3,:3].T@T[:3,:3])-1)/2,-1,1)))) for T in matrices)
    observations=[]; offsets=np.linspace(-.3,.3,31)
    for row in load(root,cal['depth'],limit=90):
        with np.load(root/row[-1],allow_pickle=False) as data:stamps=data['stamps']
        depth_stamp,lidar_stamp=map(float,stamps[1:3])
        # Keep a common time range for all tested offsets; never extrapolate.
        if any(poses.at(t) is None for t in [depth_stamp+float(dt) for dt in offsets]+[lidar_stamp]):continue
        before,after=poses.at(depth_stamp-.3),poses.at(depth_stamp+.3)
        local_translation=float(np.linalg.norm(after[:3,3]-before[:3,3]))
        local_rotation=float(np.degrees(np.arccos(np.clip((np.trace(before[:3,:3].T@after[:3,:3])-1)/2,-1,1))))
        # Static observations do not constrain timing and must not dominate a
        # seemingly sharp minimum caused by gating/noise.
        if local_translation<.02 and local_rotation<1:continue
        observations.append((row,depth_stamp,lidar_stamp))
    if len(observations)>30:
        observations=[observations[i] for i in np.linspace(0,len(observations)-1,30,dtype=int)]
    curve=[]
    for offset in offsets:
        errors=[];frames=0
        for row,depth_stamp,lidar_stamp in observations:
            camera_pose=poses.at(depth_stamp+float(offset));lidar_pose=poses.at(lidar_stamp)
            if camera_pose is None or lidar_pose is None:continue
            T=relative_camera_transform(extrinsic,camera_pose,lidar_pose)
            _,_,error=correspondences([row],T,cal['depth'])
            if len(error)>=100:errors.extend(np.abs(error).tolist());frames+=1
        curve.append(dict(camera_offset_s=float(offset),frames=frames,inliers=len(errors),
                          median_abs_plane_m=float(np.median(errors)) if errors else None))
    peak=max((r['inliers'] for r in curve),default=0)
    eligible=[r for r in curve if r['frames']>=10 and r['inliers']>=.8*peak and r['median_abs_plane_m'] is not None]
    best=min(eligible,key=lambda r:(r['median_abs_plane_m'],abs(r['camera_offset_s']))) if eligible else None
    near=[]
    if best:
        tolerance=max(.001,.1*best['median_abs_plane_m'])
        near=[r['camera_offset_s'] for r in eligible if r['median_abs_plane_m']<=best['median_abs_plane_m']+tolerance]
    observable=bool(best and (displacement>.2 or rotation>10) and len(observations)>=10 and
                    abs(best['camera_offset_s'])<.29 and max(near)-min(near)<=.06)
    report=dict(accepted=False,scope='Host-observation time-offset diagnostic, not native exposure synchronization acceptance',
                motion_reference='Raw odometry for relative-time compensation only; cumulative drift is not physical travel',
                camera_serial=cal.get('provenance',{}).get('camera_serial') if isinstance(cal.get('provenance'),dict) else None,
                maximum_motion_m=displacement,maximum_rotation_deg=rotation,common_observations=len(observations),
                offset_observable=observable,best=best,near_minimum_offset_range_s=[min(near),max(near)] if near else None,
                warning='Occlusion gates and scan motion affect residuals; a narrow minimum is not a calibrated uncertainty bound. No flags or live configuration were changed.',curve=curve)
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='curve'},indent=2))


if __name__=='__main__':main()
