"""Profile equivalent Pillow text overlays and JPEG encoding, without PL access."""
import argparse,hashlib,io,json,time
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--image',required=True);p.add_argument('--out',required=True);a=p.parse_args()
rgb=np.asarray(Image.open(a.image).convert('RGB'));font=ImageFont.load_default();rows=[]
for rep in range(30):
 outputs={}
 for cached in ((False,True) if rep%2==0 else (True,False)):
  start=time.perf_counter();im=Image.fromarray(rgb);draw=ImageDraw.Draw(im);draw.text((8,8),'Stationary replay / synthetic identity pose',fill=(255,170,40),font=font if cached else None);drawn=time.perf_counter()
  b=io.BytesIO();im.save(b,format='JPEG',quality=88,subsampling=0);encoded=time.perf_counter()
  outputs[cached]=b.getvalue();rows.append(dict(repeat=rep,cached_font=cached,compose_ms=(drawn-start)*1000,jpeg_ms=(encoded-drawn)*1000,sha256=hashlib.sha256(outputs[cached]).hexdigest()))
 if outputs[False]!=outputs[True]:raise RuntimeError('font reuse changed encoded output')
report=dict(scope='board display profile, identical JPEG settings, no PL or full-chain speedup claim',all_jpegs_exact=True,summary={str(c):{k:float(np.mean([r[k] for r in rows if r['cached_font']==c])) for k in ('compose_ms','jpeg_ms')} for c in (False,True)},rows=rows)
with Path(a.out).open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report['summary']),flush=True)
