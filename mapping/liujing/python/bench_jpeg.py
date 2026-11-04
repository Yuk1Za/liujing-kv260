"""Measure identical JPEG quality/sampling options on the board."""
import argparse,hashlib,io,json,time
from pathlib import Path
import cv2,numpy as np
from PIL import Image
from viewer import encode_frame
p=argparse.ArgumentParser();p.add_argument('image');p.add_argument('--out',required=True);a=p.parse_args();im=Image.open(a.image).convert('RGB');data=np.asarray(im);rows=[];encoded={}
for repeat in range(25):
 for backend in (('pillow','opencv') if repeat%2==0 else ('opencv','pillow')):
  start=time.perf_counter()
  if backend=='pillow':jpeg=encode_frame(im)
  else:
   ok,b=cv2.imencode('.jpg',cv2.cvtColor(np.asarray(im),cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,88,cv2.IMWRITE_JPEG_SAMPLING_FACTOR,cv2.IMWRITE_JPEG_SAMPLING_FACTOR_444]);assert ok;jpeg=b.tobytes()
  elapsed=(time.perf_counter()-start)*1000
  if repeat>=5:rows.append(dict(backend=backend,ms=elapsed))
  encoded[backend]=jpeg
decoded={k:np.asarray(Image.open(io.BytesIO(v))) for k,v in encoded.items()}
r=dict(timing_ms={b:{'mean':float(np.mean([x['ms'] for x in rows if x['backend']==b])),'p99':float(np.percentile([x['ms'] for x in rows if x['backend']==b],99))} for b in encoded},decoded_exact=bool(np.array_equal(decoded['pillow'],decoded['opencv'])),encoded_exact=encoded['pillow']==encoded['opencv'],bytes={k:len(v) for k,v in encoded.items()},sha256={k:hashlib.sha256(v).hexdigest() for k,v in encoded.items()},rows=rows)
Path(a.out).write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k!='rows'}))
