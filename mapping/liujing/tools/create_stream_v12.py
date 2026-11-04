"""Create a separately validated four-lane area experiment; never edit v11."""
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[1]
dst = root / 'hardware/stream_v12'
dst.mkdir()
for name in ('raster.cpp', 'raster.hpp', 'tb_stress.cpp', 'build.tcl'):
    shutil.copyfile(root / 'hardware/stream_v11' / name, dst / name)
p = dst / 'raster.cpp'
s = p.read_text()
old = 'struct Desc {Surfel s;uint32_t id;float numerator,radius2,inverse_radius2;uint32_t r,g,b;bool end;};\nstruct Request {Desc d;float rx,ry;ap_uint<8> pixel;bool end;};'
new = '''// Preserve the 64-byte external descriptor ABI. Internal streams carry only
// fields consumed downstream, with no float conversion or precision change.
struct Shape {float x,y,z,nx,ny,nz,radius,weight;};
struct Payload {Shape s;uint32_t id;float numerator,radius2,inverse_radius2;uint32_t r,g,b;};
struct Desc {Payload p;int32_t x0,x1,y0,y1;bool end;};
struct Request {Payload d;float rx,ry;ap_uint<8> pixel;bool end;};'''
assert old in s
s = s.replace(old, new)
old = '''  Desc d;d.id=count<=CACHE?cached_ids[j]:indices[begin+j];d.s=count<=CACHE?cached[j]:cached_surfel(world,d.id,frame_cache,tags);
  d.numerator=(d.s.nx*d.s.x+d.s.ny*d.s.y)+d.s.nz*d.s.z;d.radius2=d.s.radius*d.s.radius;
  d.inverse_radius2=1.f/d.radius2;
  d.r=(uint32_t)(d.s.r*4096.f+.5f);d.g=(uint32_t)(d.s.g*4096.f+.5f);d.b=(uint32_t)(d.s.b*4096.f+.5f);d.end=false;'''
new = '''  Desc d;d.p.id=count<=CACHE?cached_ids[j]:indices[begin+j];Surfel s=count<=CACHE?cached[j]:cached_surfel(world,d.p.id,frame_cache,tags);
  d.p.s={s.x,s.y,s.z,s.nx,s.ny,s.nz,s.radius,s.weight};
  d.x0=(int32_t)bits(s.submap);d.x1=(int32_t)bits(s.pad0);d.y0=(int32_t)bits(s.pad1);d.y1=(int32_t)bits(s.pad2);
  d.p.numerator=(s.nx*s.x+s.ny*s.y)+s.nz*s.z;d.p.radius2=s.radius*s.radius;
  d.p.inverse_radius2=1.f/d.p.radius2;
  d.p.r=(uint32_t)(s.r*4096.f+.5f);d.p.g=(uint32_t)(s.g*4096.f+.5f);d.p.b=(uint32_t)(s.b*4096.f+.5f);d.end=false;'''
assert old in s
s = s.replace(old, new)
for old, new in [('(int32_t)bits(d.s.submap)','d.x0'),('(int32_t)bits(d.s.pad0)','d.x1'),('(int32_t)bits(d.s.pad1)','d.y0'),('(int32_t)bits(d.s.pad2)','d.y1'),('Request r;r.d=d;','Request r;r.d=d.p;'),('const Surfel&s=r.d.s;','const Shape&s=r.d.s;'),('variable=ds type=fifo impl=lutram','variable=ds type=fifo impl=bram'),('variable=fs type=fifo impl=lutram','variable=fs type=fifo impl=bram')]:
    assert old in s, old
    s = s.replace(old, new)
s = s.replace('// Radius squared must be a positive normal finite FP32 value; PS validates it.', '// Radius squared and its reciprocal must be normal FP32; PS validates the domain.')
p.write_text(s)
p = dst / 'build.tcl'
p.write_text(p.read_text().replace('two-lane int_bounds_v1 candidate','four-lane compact-payload/BRAM-FIFO candidate').replace('-DLIUJING_LANES=2','-DLIUJING_LANES=4'))
print(dst)
