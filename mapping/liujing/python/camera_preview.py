"""Single-frame RGB-D surface preview, explicitly NOT moving SLAM acceptance."""
import argparse,json,time,platform
from pathlib import Path
import numpy as np
from PIL import Image
from native import Map,Camera
from alignment import register_color,checked_pose
def main():
 p=argparse.ArgumentParser();p.add_argument('capture');p.add_argument('--out',required=True);p.add_argument('--translation-unit',choices=['mm'],required=True);a=p.parse_args()
 root=Path(a.capture);out=Path(a.out);out.mkdir(parents=True,exist_ok=True);cal=json.loads((root/'factory_calibration.json').read_text())
 if cal['mirrored']:raise ValueError('mirrored factory calibration needs explicit treatment')
 T=np.eye(4);T[:3,:3]=np.array(cal['rotation_raw']).reshape(3,3);T[:3,3]=np.array(cal['translation_raw'])*.001;checked_pose(T);cal['T_color_depth']=T.tolist()
 d=np.asarray(Image.open(root/'0_depth.png'),dtype=np.float32)*.001;rgb=np.asarray(Image.open(root/'0_rgb.png').convert('RGB'));d,rgb=register_color(d,rgb,cal)
 cam=Camera.from_dict(cal['depth']);m=Map();start=time.perf_counter();m.integrate(d,rgb,cam,np.eye(4),1);fused=time.perf_counter();im,dep,ids=m.render(cam,np.eye(4));done=time.perf_counter();Image.fromarray(im).save(out/'surface.png');Image.fromarray(rgb).save(out/'registered_rgb.png')
 shifted=np.eye(4);shifted[0,3]=.12;shifted[2,3]=-.15;novel,_,_=m.render(cam,shifted);Image.fromarray(novel).save(out/'novel_view.png');np.save(out/'surfels.npy',m.surfels())
 report={'mode':'single_real_rgbd_frame_camera_coordinates','machine':platform.machine(),'backend':'native_cpu_fp32','pose_source':'identity for one frame only; not lidar registration','factory_translation_unit_assumption':'mm; verify RGB/depth edges before live acceptance','surfels':m.count,'fusion_ms':(fused-start)*1000,'render_ms':(done-fused)*1000,'coverage':float(np.mean(ids>=0)),'hardware_acceleration':False};(out/'report.json').write_text(json.dumps(report,indent=2));(out/'camera.json').write_text(json.dumps(cam.as_dict()));print(json.dumps(report))
if __name__=='__main__':main()
