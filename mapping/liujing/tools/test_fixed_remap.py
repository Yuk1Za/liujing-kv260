"""Pixel equivalence of cached integer maps on captured real RGB-D frames."""
import argparse,json,pathlib,cv2,numpy as np
p=argparse.ArgumentParser();p.add_argument('capture',type=pathlib.Path);p.add_argument('calibration',type=pathlib.Path);a=p.parse_args();cal=json.loads(a.calibration.read_text());result={}
for key in ('depth','color'):
 c=cal[key];K=np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
 size=(320,200) if key=='depth' else (640,480);N=K.copy();nearest=key=='depth'
 if nearest:N[0,0]*=.5;N[1,1]*=.5;N[0,2]=(K[0,2]+.5)*.5-.5;N[1,2]=(K[1,2]+.5)*.5-.5
 x,y=cv2.initUndistortRectifyMap(K,np.array(c['distortion']),None,N,size,cv2.CV_32FC1);m,n=cv2.convertMaps(x,y,cv2.CV_16SC2,nninterpolation=nearest);flag=cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR
 mismatch=0;sparse_mismatch=0;frames=0
 for f in a.capture.glob('capture_*.depth' if nearest else 'capture_*.jpg'):
  image=np.fromfile(f,dtype=np.uint16).reshape(400,640) if nearest else cv2.imread(str(f));old=cv2.remap(image,x,y,flag);new=cv2.remap(image,m,n,flag);mismatch+=int(np.count_nonzero(old!=new));frames+=1
  if not nearest:
   # A fixed permutation covers 64000 arbitrary destination pixels, including
   # border pixels. Gathered maps must reproduce full remap channel samples.
   indices=(np.arange(64000,dtype=np.int64)*1543)%(640*480)
   selected_m=m.reshape(-1,2)[indices].reshape(200,320,2);selected_n=n.reshape(-1)[indices].reshape(200,320)
   sparse=cv2.remap(image,selected_m,selected_n,flag)
   sparse_mismatch+=int(np.count_nonzero(sparse.reshape(-1,3)!=new.reshape(-1,3)[indices]))
 result[key]={'frames':frames,'mismatched_channel_samples':mismatch,'sparse_mismatched_channel_samples':sparse_mismatch}
print(json.dumps(result,indent=2));assert all(r['frames'] and r['mismatched_channel_samples']==0 and r['sparse_mismatched_channel_samples']==0 for r in result.values())
