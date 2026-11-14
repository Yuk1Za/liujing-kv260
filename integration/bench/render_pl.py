import sys, time, struct, ctypes as C, numpy as np
sys.path.insert(0, "/home/ubuntu/liujing/python")
from native import Map, Camera, ptr
from pynq import Overlay, allocate

REG = dict(camera_surfels=0x10, offsets=0x18, indices=0x20, depth=0x28, ids=0x30, rgb=0x38,
           width=0x40, height=0x48, fx=0x50, fy=0x58, cx=0x60, cy=0x68,
           near_z=0x70, far_z=0x78, blend_band=0x80, mode=0x88)
D = "/home/ubuntu/liujing/runs/motion_alignment_20261026_01"
BIT = "/home/ubuntu/fusion_probe_pl/fusion.bit"
SCAP, ICAP = 200000, 4000000

cam = Camera(640, 480, 453.6891, 453.6891, 328.8929, 243.7859, 0.1, 8.0)
T = np.eye(4, dtype=np.float32)
m = Map(capacity=500000, stride=2, library="/home/ubuntu/liujing/build_guarded/libliujing.so", threads=4)
for i in range(4):
    z = np.load("%s/%03d.npz" % (D, i))
    dep = np.zeros((480, 640), np.float32); dep[:400] = z["depth"]
    m.integrate(dep, z["rgb"], cam, T, float(i) * 0.1)
count = m.count
print("surfels = %d" % count, flush=True)

t0 = time.perf_counter(); imc, dc, idc = m.render(cam, T); t_cpu = (time.perf_counter() - t0) * 1000; t_cpu_end = time.perf_counter()

ol = Overlay(BIT); ip = ol.lj_raster_0
surf = allocate(shape=(SCAP, 16), dtype=np.float32)
offs = allocate(shape=(1201,), dtype=np.uint32)
idxs = allocate(shape=(ICAP,), dtype=np.uint32)
depb = allocate(shape=(307200,), dtype=np.float32)
idsb = allocate(shape=(307200,), dtype=np.int32)
rgbb = allocate(shape=(307200,), dtype=np.uint32)

f = m.lib.lj_prepare_checked
f.argtypes = [C.c_void_p, C.c_void_p, C.c_void_p, C.c_void_p, C.c_void_p, C.c_void_p, C.c_int, C.c_int, C.c_uint64, C.c_int]
f.restype = C.c_int
ver = m.version
refs = f(m.handle, C.byref(cam), ptr(T), ptr(surf), ptr(offs), ptr(idxs), ICAP, SCAP, ver, 0)
print("prepare refs = %d" % refs, flush=True)
prep = time.perf_counter()

tiles = ((cam.width + 15)//16) * ((cam.height + 15)//16)
surf[:max(1, count)].flush(); offs[:tiles+1].flush()
if refs: idxs[:refs].flush()

for key, buf in [("camera_surfels", surf), ("offsets", offs), ("indices", idxs),
                 ("depth", depb), ("ids", idsb), ("rgb", rgbb)]:
    a = int(buf.physical_address)
    ip.write(REG[key], a & 0xFFFFFFFF); ip.write(REG[key] + 4, a >> 32)
    lo = int(ip.read(REG[key])); hi = int(ip.read(REG[key] + 4))
    got = lo | (hi << 32)
    print("  %-14s want=%x back=%x %s" % (key, a, got, "OK" if got == a else "MISMATCH"), flush=True)
ip.write(REG["width"], cam.width); ip.write(REG["height"], cam.height)
ip.write(REG["mode"], 0)
for key in ("fx","fy","cx","cy","near_z","far_z","blend_band"):
    v = 0.01 if key == "blend_band" else getattr(cam, key)
    ip.write(REG[key], struct.unpack("<I", struct.pack("<f", v))[0])
print("CTRL before=%x" % int(ip.read(0x00)), flush=True)
ip.read(0x00); sub = time.perf_counter(); ip.write(0x00, 1)
time.sleep(0.01); print("CTRL after 10ms=%x" % int(ip.read(0x00)), flush=True)
dl = sub + 30; ok = False
while time.perf_counter() < dl:
    if int(ip.read(0x00)) & 2: ok = True; break
    time.sleep(0.00005)
fin = time.perf_counter(); print("CTRL final=%x" % int(ip.read(0x00)), flush=True)
for b in (depb, idsb, rgbb): b.invalidate()
d_p = np.array(depb).reshape(480, 640); id_p = np.array(idsb).reshape(480, 640)
rgb_p = np.array(rgbb).reshape(480, 640)
end = time.perf_counter()
print("ap_done=%s" % ok, flush=True)
print("CPU render %.1f ms | PL: prepare %.1f  submit %.1f  pl_wait %.1f  readback %.1f  total %.1f ms" % (
    t_cpu, (prep-t_cpu_end)*1000, (sub-prep)*1000, (fin-sub)*1000, (end-fin)*1000, (end-t0)*1000), flush=True)
print("ids>=0 CPU=%d PL=%d  ids_equal=%s" % (int((idc>=0).sum()), int((id_p>=0).sum()), np.array_equal(idc, id_p)), flush=True)
print("depth maxdiff=%.6f  mean|diff|=%.6f" % (np.abs(dc-d_p).max(), np.abs(dc-d_p).mean()), flush=True)
