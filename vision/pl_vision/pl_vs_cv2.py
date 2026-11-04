#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PL FAST + 稀疏 LK 与 cv2 对拍 v2

相对 v1 的两处修正（v1 有取样偏差，导致后期帧分歧虚高）：
  1. 角点只在「有效图像区」内取 —— 等比缩放后上下有黑边（640x480 里只有中间 193 行是图），
     v1 只按 PMAX=10 裁剪，取到的点会贴着黑边，LK 窗口跨到黑边 -> 不稳定
  2. 取样改为「沿序列均匀间隔」而非「取前 N 个」—— 后者按 raster 顺序取，点全挤在图像上部
"""
import glob
import time

import numpy as np
import cv2
from pynq import Overlay, allocate

BIT = "/home/ubuntu/merge/merged.bit"
F_AP, F_IN, F_TH, F_OUT, F_R, F_C = 0x00, 0x10, 0x1c, 0x24, 0x30, 0x38
S_AP, S_PREV, S_CUR, S_COR, S_FLOW, S_ST = 0x00, 0x10, 0x1c, 0x28, 0x34, 0x40
S_N, S_ROWS, S_COLS, S_ITER = 0x4c, 0x54, 0x5c, 0x64
W, H = 640, 480
WH, SEARCH = 5, 4
PMAX = WH + SEARCH + 1
Q = 6
MAXIT = 20
TH = 20
NC = 100
IMG_DIR = "/home/ubuntu/kitti_cam/image_02/data"


def wr64(mm, a, v):
    mm.write(a, v & 0xFFFFFFFF)
    mm.write(a + 4, (v >> 32) & 0xFFFFFFFF)


def pack(img):
    r, c = img.shape
    return img.reshape(r, c // 4, 4).copy().view(np.uint32).reshape(r, c // 4)


def grab(p):
    """与节点完全一致：等比缩放 + 上下补零 + 灰度。返回 (gray, ytop, ybot) 有效行区间。"""
    img = cv2.imread(p, cv2.IMREAD_COLOR)
    h0, w0 = img.shape[:2]
    h2 = min(H, max(1, int(round(h0 * W / float(w0)))))
    small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((H, W, 3), np.uint8)
    top = (H - h2) // 2
    canvas[top:top + h2] = small
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY), top, top + h2


ol = Overlay(BIT, download=False)
print("PL IP:", sorted(ol.ip_dict))
fm = ol.fast_accel_0.mmio
sm = ol.slk_accel_0.mmio

inp = allocate(shape=(H, W), dtype=np.uint8, cacheable=1)
out = allocate(shape=(H, W), dtype=np.uint8, cacheable=1)
pb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cor_b = allocate(shape=(NC,), dtype=np.uint32, cacheable=1)
fl_b = allocate(shape=(NC,), dtype=np.uint32, cacheable=1)
st_b = allocate(shape=(NC,), dtype=np.uint32, cacheable=1)
wr64(fm, F_IN, inp.device_address)
wr64(fm, F_OUT, out.device_address)
fm.write(F_TH, TH)
fm.write(F_R, H)
fm.write(F_C, W)
wr64(sm, S_PREV, pb.device_address)
wr64(sm, S_CUR, cb.device_address)
wr64(sm, S_COR, cor_b.device_address)
wr64(sm, S_FLOW, fl_b.device_address)
wr64(sm, S_ST, st_b.device_address)
sm.write(S_ROWS, H)
sm.write(S_COLS, W)
sm.write(S_ITER, MAXIT)


def pl_fast(img):
    inp[:] = img
    inp.sync_to_device()
    out.fill(0)
    out.sync_to_device()
    t0 = time.perf_counter()
    fm.write(F_AP, 1)
    while not ((fm.read(F_AP) >> 1) & 1):
        time.sleep(0.0002)
    dt = time.perf_counter() - t0
    out.sync_from_device()
    ys, xs = np.nonzero(np.array(out))
    return np.stack([xs, ys], 1).astype(np.int32), dt


def pl_lk(prev, cur, pts):
    N = len(pts)
    pb[:] = pack(prev)
    pb.sync_to_device()
    cb[:] = pack(cur)
    cb.sync_to_device()
    cor = (pts[:, 1].astype(np.uint32) << 16) | (pts[:, 0].astype(np.uint32) & 0xFFFF)
    cor_b[:N] = cor
    cor_b.sync_to_device()
    sm.write(S_N, N)
    t0 = time.perf_counter()
    sm.write(S_AP, 1)
    while not ((sm.read(S_AP) >> 1) & 1):
        time.sleep(0.0002)
    dt = time.perf_counter() - t0
    fl_b.sync_from_device()
    st_b.sync_from_device()
    raw = np.array(fl_b).view(np.int16).reshape(NC, 2)
    pu = raw[:N, 1].astype(np.float32) / (1 << Q)
    pv = raw[:N, 0].astype(np.float32) / (1 << Q)
    return pu, pv, np.array(st_b)[:N], dt


def pick(allp, ytop, ybot, n=NC):
    """只在有效图像区内取角点；若多于此数，沿序列均匀间隔取样（而非取前 n 个）"""
    m = ((allp[:, 0] >= PMAX) & (allp[:, 0] < W - PMAX) &
         (allp[:, 1] >= ytop + PMAX) & (allp[:, 1] < ybot - PMAX))
    sel = allp[m]
    if len(sel) > n:
        sel = sel[np.linspace(0, len(sel) - 1, n).astype(np.int64)]
    return sel, int(m.sum())


def run_case(tag, img0, img1, ytop, ybot, dx=None, dy=None):
    print("\n===== %s =====" % tag)
    allp, t_fast = pl_fast(img0)
    pts, n_valid = pick(allp, ytop, ybot)
    N = len(pts)
    kps = cv2.FastFeatureDetector_create(threshold=TH, nonmaxSuppression=True).detect(img0, None)
    print("  FAST: PL %d 个（有效区内 %d，取样 %d） | cv2 %d 个 | PL %.2f ms"
          % (len(allp), n_valid, N, len(kps), t_fast * 1e3))
    if N == 0:
        return
    pu, pv, st, dt = pl_lk(img0, img1, pts)
    lkp = dict(winSize=(11, 11), maxLevel=1,
               criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, MAXIT, 0.01))
    p1, stc, _ = cv2.calcOpticalFlowPyrLK(
        img0, img1, pts.reshape(-1, 1, 2).astype(np.float32), None, **lkp)
    cu = p1[:, 0, 0] - pts[:, 0]
    cvv = p1[:, 0, 1] - pts[:, 1]
    d = np.sqrt((pu - cu) ** 2 + (pv - cvv) ** 2)
    print("  LK  : PL %.2f us/点（总 %.2f ms, N=%d）" % (dt * 1e6 / N, dt * 1e3, N))
    print("  PL  位移中位 (u,v) = (%.3f, %.3f) | cv2 = (%.3f, %.3f)"
          % (np.median(pu), np.median(pv), np.median(cu), np.median(cvv)))
    print("  |PL-cv2|: 中位 %.3f px, 90%% %.3f px, >1px 占比 %.0f%%"
          % (np.median(d), np.percentile(d, 90), 100 * (d > 1).mean()))
    sv = st[st < 100000]
    print("  status: ==0 占比 %.0f%% | 范围 [%d, %d] | 众数 %s"
          % (100 * (st == 0).mean(), st.min() if st.size else -1, st.max() if st.size else -1,
             np.bincount(sv).argmax() if sv.size else '-'))
    if dx is not None:
        e = np.sqrt((pu - dx) ** 2 + (pv - dy) ** 2)
        ec = np.sqrt((cu - dx) ** 2 + (cvv - dy) ** 2)
        print("  对真值(%.1f,%.1f): PL 误差中位 %.3f px | cv2 %.3f px | PL>1px %.0f%%"
              % (dx, dy, np.median(e), np.median(ec), 100 * (e > 1).mean()))


files = sorted(glob.glob(IMG_DIR + "/*.png"))
print("图序列 %d 帧" % len(files))
g0, yt0, yb0 = grab(files[0])
g1, _, _ = grab(files[1])
g2, _, _ = grab(files[2])
print("有效行区间: %d .. %d （共 %d 行）" % (yt0, yb0, yb0 - yt0))

run_case("(a) 真实相邻帧 0->1", g0, g1, yt0, yb0)
run_case("(a2) 真实相邻帧 1->2", g1, g2, yt0, yb0)

w1 = cv2.warpAffine(g0, np.float32([[1, 0, 3.0], [0, 1, 2.0]]), (W, H),
                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
run_case("(b) 人工 warp 小位移 (3.0, 2.0)", g0, w1, yt0, yb0, 3.0, 2.0)

w2 = cv2.warpAffine(g0, np.float32([[1, 0, 8.0], [0, 1, 6.0]]), (W, H),
                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
run_case("(c) 人工 warp 大位移 (8.0, 6.0)", g0, w2, yt0, yb0, 8.0, 6.0)

w3 = cv2.warpAffine(g0, np.float32([[1, 0, 5.0], [0, 1, 4.0]]), (W, H),
                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
run_case("(c2) 人工 warp 中位移 (5.0, 4.0)", g0, w3, yt0, yb0, 5.0, 4.0)

for k in (300, 600, 900, 1050):
    if k + 1 < len(files):
        ga, yt, yb = grab(files[k])
        gb, _, _ = grab(files[k + 1])
        run_case("(d%d) 真实相邻帧 %d->%d" % (k, k, k + 1), ga, gb, yt, yb)
print("\nDONE")
