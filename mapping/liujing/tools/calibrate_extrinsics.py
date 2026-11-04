"""Estimate an extrinsic candidate from measured 3-D LiDAR / 2-D RGB matches.

Does not mark live calibration accepted. Needs separate held-out observations.
Input NPZ: lidar_xyz (N,3) metres, color_uv (N,2) pixels.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from alignment import checked_pose
def main():
 import cv2
 p=argparse.ArgumentParser();p.add_argument('correspondences');p.add_argument('--calibration',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 dest=Path(a.out)
 if dest.exists():raise ValueError('output exists')
 cal=json.loads(Path(a.calibration).read_text());data=np.load(a.correspondences,allow_pickle=False);xyz=np.asarray(data['lidar_xyz'],np.float64);uv=np.asarray(data['color_uv'],np.float64)
 if xyz.ndim!=2 or xyz.shape[1]!=3 or uv.shape!=(len(xyz),2) or len(xyz)<12 or not np.isfinite(xyz).all() or not np.isfinite(uv).all():raise ValueError('at least 12 finite measured matches required')
 if np.linalg.matrix_rank(xyz-xyz.mean(0),tol=.01)<3:raise ValueError('need noncoplanar observations to avoid ambiguous calibration')
 c=cal['color'];K=np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1]],float);dist=np.array(c.get('distortion',[0]*8),float)
 ok,rvec,tvec,inliers=cv2.solvePnPRansac(xyz,uv,K,dist,iterationsCount=500,reprojectionError=2.,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
 if not ok or inliers is None or len(inliers)<12:raise ValueError('insufficient geometric consensus')
 idx=inliers.ravel();rvec,tvec=cv2.solvePnPRefineLM(xyz[idx],uv[idx],K,dist,rvec,tvec);R,_=cv2.Rodrigues(rvec);T=np.eye(4);T[:3,:3]=R;T[:3,3]=tvec.ravel();checked_pose(T)
 predicted,_=cv2.projectPoints(xyz,rvec,tvec,K,dist);errors=np.linalg.norm(predicted.reshape(-1,2)-uv,axis=1)
 # T_color_lidar^-1 * T_color_depth maps depth-optical coordinates into LiDAR.
 cal['T_pose_depth']=(np.linalg.inv(T)@checked_pose(cal['T_color_depth'])).tolist();cal.setdefault('verified',{})['pose_camera']=False
 cal['extrinsic_candidate_report']={'inliers':idx.tolist(),'all_reprojection_px':errors.tolist(),'inlier_p95_px':float(np.percentile(errors[idx],95)),'accepted':False,'reason':'candidate only; separate held-out observations and pose-frame audit required'}
 dest.write_text(json.dumps(cal,indent=2));print(cal['extrinsic_candidate_report'])
if __name__=='__main__':main()
