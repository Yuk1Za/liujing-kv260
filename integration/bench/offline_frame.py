import numpy as np, math, time, sqlite3, struct, subprocess
from pynq import Overlay, allocate

BAG="/home/ubuntu/liujing/runs/online_20261010_225345_838bed/bag/bag_0.db3"
BIT="/home/ubuntu/fusion_probe_pl/fusion.bit"
HS, MAXP, MINR = 1800, 115200, 0.5

def parse_pc2(b):
    o = 4
    def al(o, n): return (o + n - 1) // n * n
    o = al(o,4); o += 8
    o = al(o,4); ln = struct.unpack_from("<I", b, o)[0]; o += 4 + ln
    o = al(o,4); h, w = struct.unpack_from("<II", b, o); o += 8
    o = al(o,4); nf = struct.unpack_from("<I", b, o)[0]; o += 4
    for _ in range(nf):
        o = al(o,4); ln = struct.unpack_from("<I", b, o)[0]; o += 4 + ln
        o = al(o,4); o += 4
        o += 1
        o = al(o,4); o += 4
    o = al(o,1); o += 1
    o = al(o,4); ps, rs = struct.unpack_from("<II", b, o); o += 8
    o = al(o,4); ln = struct.unpack_from("<I", b, o)[0]; o += 4
    return h, w, ps, b[o:o+ln]

con = sqlite3.connect(BAG)
blob = bytes(con.execute("SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=? LIMIT 1", ("/point_cloud_raw",)).fetchone()[0])
raw = subprocess.run(["zstd","-d","-c"], input=blob, capture_output=True).stdout
h, w, ps, dat = parse_pc2(raw)
n = len(dat)//ps
arr = np.frombuffer(dat[:n*ps], dtype=np.uint8).reshape(n, ps)
x = arr[:,0:4].copy().view(np.float32).ravel()
y = arr[:,4:8].copy().view(np.float32).ravel()
z = arr[:,8:12].copy().view(np.float32).ravel()
print("frame: h=%d w=%d pt_step=%d n=%d x=[%.2f,%.2f]" % (h,w,ps,n,x.min(),x.max()), flush=True)

rng=np.sqrt(x*x+y*y+z*z)
ok=np.isfinite(rng)&(rng>=MINR)&(rng<125)&(np.abs(x)<125)&(np.abs(y)<125)&(np.abs(z)<125)
x,y,z=x[ok],y[ok],z[ok]
print("after filter: %d" % len(x), flush=True)
col=np.clip(((np.arctan2(y,x)+np.pi)/(2*np.pi)*HS).astype(np.int32),0,HS-1)
rr=np.maximum(np.sqrt(x*x+y*y),1e-6)
elev=np.degrees(np.arctan2(z,rr))
scan=np.clip(np.rint((elev+15.0)*0.5),0,15).astype(np.int32)
order=np.argsort(col,kind="stable")
xs,ys,zs=x[order],y[order],z[order]; cs=col[order]; ss=scan[order]
idx32=(0x3F800000|(ss.astype(np.uint32)<<11)|(cs.astype(np.uint32)&0x7FF)).astype(np.uint32)
fi=idx32.view(np.float32); N=len(xs)
print("reordered N=%d  per-scan=%s" % (N,np.bincount(ss,minlength=16).tolist()), flush=True)

pts=np.zeros((MAXP,4),dtype=np.float32)
pts[:N,0]=xs; pts[:N,1]=ys; pts[:N,2]=zs; pts[:N,3]=fi

ol=Overlay(BIT); ip=ol.fpga_feature_extract_0; rm=ip.register_map
inb=allocate(shape=(MAXP,4),dtype=np.float32)
pob=allocate(shape=(MAXP,4),dtype=np.float32)
rab=allocate(shape=(MAXP,),dtype=np.float32)
lfb=allocate(shape=(MAXP,),dtype=np.uint8)
grb=allocate(shape=(MAXP,),dtype=np.uint8)
flb=allocate(shape=(6*64*4,2),dtype=np.int16)
lsb=allocate(shape=(6*64*20,2),dtype=np.int16)
shb=allocate(shape=(6*64*2,2),dtype=np.int16)
inb[:]=pts
for nm,b in [("laserCloudInArray",inb),("rangeimage_point",pob),("rangeimage_range",rab),
             ("rangeimage_lessflat",lfb),("rangeimage_ground",grb),
             ("rangeimage_flat",flb),("rangeimage_lesssharp",lsb),("rangeimage_sharp",shb)]:
    a=int(b.physical_address); setattr(rm,nm+"_1",a&0xFFFFFFFF); setattr(rm,nm+"_2",a>>32)
a0=-math.atan2(float(ys[0]),float(xs[0])); a1=-math.atan2(float(ys[-1]),float(xs[-1]))+2*math.pi
if a1-a0>3*math.pi: a1-=2*math.pi
elif a1-a0<math.pi: a1+=2*math.pi
rm.start_point_ori=a0; rm.end_point_ori=a1; rm.cloudSize=N
inb.sync_to_device()
t0=time.time(); ip.write(0x00,1); done=False
while time.time()-t0<5.0:
    if int(ip.read(0x00))&0x02: done=True; break
    time.sleep(0.00005)
dt=(time.time()-t0)*1000
for b in (pob,rab,lfb,grb,flb,lsb,shb): b.invalidate()
def nz(a): return int((np.abs(a.astype(np.float32))>1e-9).sum())
print("ap_done=%s  %.2f ms" % (done,dt), flush=True)
print("range nonzero=%d / %d" % (int((rab[:N]>0).sum()),N), flush=True)
print("point x nonzero=%d" % nz(pob[:N,0]), flush=True)
print("lessflat nonzero=%d ground nonzero=%d" % (int((lfb[:N]>0).sum()),int((grb[:N]>0).sum())), flush=True)
print("flat=%d lesssharp=%d sharp=%d" % (int((flb!=0).any(axis=1).sum()),int((lsb!=0).any(axis=1).sum()),int((shb!=0).any(axis=1).sum())), flush=True)
print("flat[:5]=",flb[:5].tolist(),"sharp[:5]=",shb[:5].tolist(), flush=True)
