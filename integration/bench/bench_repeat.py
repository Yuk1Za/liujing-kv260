import sys, time, struct, ctypes as C, numpy as np
sys.path.insert(0, "/home/ubuntu/liujing/python")
from native import Map, Camera, ptr
from pynq import Overlay, allocate
perf = time.perf_counter
REG = dict(camera_surfels=0x10, offsets=0x18, indices=0x20, depth=0x28, ids=0x30, rgb=0x38,
           width=0x40, height=0x48, fx=0x50, fy=0x58, cx=0x60, cy=0x68,
           near_z=0x70, far_z=0x78, blend_band=0x80, mode=0x88)
D = "/home/ubuntu/liujing/runs/motion_alignment_20261026_01"
SCAP, ICAP = 200000, 4000000
cam = Camera(640, 480, 453.6891, 453.6891, 328.8929, 243.7859, 0.1, 8.0)
T = np.eye(4, dtype=np.float32)
m = Map(capacity=500000, stride=2, library="/home/ubuntu/liujing/build_guarded/libliujing.so", threads=4)
for i in range(4):
    z = np.load("%s/%03d.npz" % (D, i)); dep = np.zeros((480,640), dtype=np.float32); dep[:400] = z["depth"]
    m.integrate(dep, z["rgb"], cam, T, float(i)*0.1)
count = m.count; print("surfels=%d" % count, flush=True)

cpu = []
for _ in range(5):
    t = perf(); m.render(cam, T); cpu.append((perf()-t)*1000)
print("CPU render 5x: " + " ".join("%.1f" % v for v in cpu) + "  min=%.1f" % min(cpu), flush=True)

ol = Overlay("/home/ubuntu/fusion_probe_pl/fusion.bit"); ip = ol.lj_raster_0
surf = allocate(shape=(SCAP,16), dtype=np.float32); offs = allocate(shape=(1201,), dtype=np.uint32)
idxs = allocate(shape=(ICAP,), dtype=np.uint32); depb = allocate(shape=(307200,), dtype=np.float32)
idsb = allocate(shape=(307200,), dtype=np.int32); rgbb = allocate(shape=(307200,), dtype=np.uint32)
f = m.lib.lj_prepare_checked
f.argtypes = [C.c_void_p]*6 + [C.c_int, C.c_int, C.c_uint64, C.c_int]
f.restype = C.c_int
ver = m.version
tiles = 40*30
for key, buf in [("camera_surfels",surf),("offsets",offs),("indices",idxs),("depth",depb),("ids",idsb),("rgb",rgbb)]:
    a = int(buf.physical_address); ip.write(REG[key], a & 0xFFFFFFFF); ip.write(REG[key]+4, a >> 32)
ip.write(REG["width"], cam.width); ip.write(REG["height"], cam.height); ip.write(REG["mode"], 0)
for key in ("fx","fy","cx","cy","near_z","far_z","blend_band"):
    v = 0.01 if key == "blend_band" else getattr(cam, key)
    ip.write(REG[key], struct.unpack("<I", struct.pack("<f", v))[0])
for it in range(5):
    t0 = perf(); refs = f(m.handle, C.byref(cam), ptr(T), ptr(surf), ptr(offs), ptr(idxs), ICAP, SCAP, ver, 0); t1 = perf()
    surf[:count].flush(); offs[:tiles+1].flush(); idxs[:refs].flush(); t2 = perf()
    ip.read(0x00); ip.write(0x00, 1); t3 = perf()
    while not (int(ip.read(0x00)) & 2): time.sleep(0.00005)
    t4 = perf()
    for b in (depb, idsb, rgbb): b.invalidate()
    d = np.array(depb); t5 = perf()
    print("it%d refs=%d  prepare=%.1f flush=%.1f submit=%.1f pl_wait=%.1f readback=%.1f ms" % (
        it, refs, (t1-t0)*1000, (t2-t1)*1000, (t3-t2)*1000, (t4-t3)*1000, (t5-t4)*1000), flush=True)
