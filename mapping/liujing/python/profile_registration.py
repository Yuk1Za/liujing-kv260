import json,time
from pathlib import Path
import numpy as np
from PIL import Image
from native import Map
from alignment import register_color
root=Path('runs/camera_02');cal=json.loads((root/'factory_calibration.json').read_text());T=np.eye(4);T[:3,:3]=np.array(cal['rotation_raw']).reshape(3,3);T[:3,3]=np.array(cal['translation_raw'])*.001;cal['T_color_depth']=T.tolist();d=np.asarray(Image.open(root/'0_depth.png'),np.float32)*.001;rgb=np.asarray(Image.open(root/'0_rgb.png').convert('RGB'));m=Map();rows=[]
for i in range(6):
 p={};register_color(d,rgb,cal,native_library=m.lib,profile=p);rows.append(p)
print(json.dumps(rows));m.close()
