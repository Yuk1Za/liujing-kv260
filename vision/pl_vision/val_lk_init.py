#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判定：真实帧上 LK 幅度被压低，是「局部极小（初值问题）」还是「算术问题」。

A0 冷启 0        基线
A3 初值 = cv2 真值  若能收敛到真值 => 局部极小（初值问题），否则算术问题
A4 初值 = 上一帧中位位移（全局先验，工程上可做）
A5 初值 = 本帧中位位移（相当于 oracle 先验的上限）
"""
import os
import sys
import glob
import time

import numpy as np
import cv2

H, W, PMAX, MAXIT, Q = 480, 640, 10, 20, 6
NC, MIN_DIST, QL = 100, 20, 0.01

for d in ("/home/ubuntu", "/home/ubuntu/merge_st"):
    for f in ("_current_metadata.pkl", "global_pl_state.json"):
        p = os.path.join(d, f)
        if os.path.exists(p):
            os.remove(p)

from pynq import Overlay, allocate  # noqa: E402

ol = Overlay("/home/ubuntu/merge_st/kv260_merged_st.bit")
lk = ol.slk_accel_0
mm = lk.mmio
S_AP, S_PREV, S_CUR, S_COR, S_FLOW, S_ST = 0x00, 0x10, 0x1C, 0x28, 0x34, 0x40
S_N, S_ROWS, S_COLS, S_ITER = 0x4C, 0x54, 0x5C, 0x64


def wr64(m, a, v):
    m.write(a, v & 0xFFFFFFFF)
    m.write(a + 4, (v >> 32) & 0xFFFFFFFF)


def pack(img):
    r, c = img.shape
    return img.reshape(r, c // 4, 4).copy().view(np.uint32).reshape(r, c // 4)


pb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cor_b = allocate(shape=(NC,), dtype=np.uint32, cacheable=1)
fl_b = allocate(shape=(NC,), dtype=np.uint32, cacheable=1)
st_b = allocate(shape=(NC,), dtype=np.uint32, cacheable=1)
for a, b in ((S_PREV, pb), (S_CUR, cb), (S_COR, cor_b), (S_FLOW, fl_b), (S_ST, st_b)):
    wr64(mm, a, b.device_address)
mm.write(S_ROWS, H)
mm.write(S_COLS, W)
mm.write(S_ITER, MAXIT)


def run(prev, cur, pts, f0=None):
    np.copyto(np.asarray(pb), pack(prev)); pb.sync_to_device()
    np.copyto(np.asarray(cb), pack(cur)); cb.sync_to_device()
    n = len(pts)
    cor_b[:n] = (pts[:, 1].astype(np.uint32) << 16) | (pts[:, 0].astype(np.uint32) & 0xFFFF)
    if f0 is None:
        fl_b[:n] = 0
    else:
        fu = np.clip(np.round(f0[:, 0] * (1 << Q)), -32768, 32767).astype(np.int32)
        fv = np.clip(np.round(f0[:, 1] * (1 << Q)), -32768, 32767).astype(np.int32)
        fl_b[:n] = ((fu.astype(np.uint32) & 0xFFFF) << 16) | (fv.astype(np.uint32) & 0xFFFF)
    cor_b.sync_to_device(); fl_b.sync_to_device()
    mm.write(S_N, n)
    t0 = time.perf_counter()
    mm.write(S_AP, 1)
    while not ((mm.read(S_AP) >> 1) & 1):
        pass
    dt = time.perf_counter() - t0
    fl_b.sync_from_device(); st_b.sync_from_device()
    raw = np.array(fl_b).view(np.int16).reshape(NC, 2)[:n]
    return (raw[:, 1].astype(np.float32) / (1 << Q),
            raw[:, 0].astype(np.float32) / (1 << Q),
            np.array(st_b)[:n].astype(np.uint8), dt)


def grab(p):
    img = cv2.imread(p, cv2.IMREAD_COLOR)
    h0, w0 = img.shape[:2]
    h2 = min(H, max(1, int(round(h0 * W / float(w0)))))
    small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((H, W, 3), img.dtype)
    top = (H - h2) // 2
    canvas[top:top + h2] = small
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)


MASK = np.zeros((H, W), np.uint8)
MASK[PMAX:H - PMAX, PMAX:W - PMAX] = 255


def gftt(g):
    p = cv2.goodFeaturesToTrack(g, NC, QL, MIN_DIST, mask=MASK, blockSize=3)
    return np.round(p.reshape(-1, 2)).astype(np.int32) if p is not None else np.zeros((0, 2), np.int32)


def rep(tag, u, v, st, refu, refv, dt):
    ok = (st == 1) & np.isfinite(refu)
    if ok.sum() < 5:
        print("  %-26s ok=%d 太少" % (tag, int(ok.sum())))
        return
    du, dv = u[ok] - refu[ok], v[ok] - refv[ok]
    A = np.stack([refu[ok], np.ones(ok.sum())], 1)
    a = np.linalg.lstsq(A, u[ok], rcond=None)[0][0]
    r = np.corrcoef(u[ok], refu[ok])[0, 1]
    sat = float((np.abs(u) >= 7.9).mean())
    print("  %-26s ok %3d | med u %+6.2f (ref %+6.2f) | p90|err| %5.2f | r %+5.3f | 斜率 %4.2f | 饱和 %.0f%% | %.2f ms"
          % (tag, int(ok.sum()), float(np.median(u[ok])), float(np.median(refu[ok])),
             float(np.percentile(np.hypot(du, dv), 90)), r, a, sat * 100, dt * 1e3))
    sys.stdout.flush()


files = sorted(glob.glob("/home/ubuntu/kitti_cam/image_02/data/*.png"))
prev_med = None
for k in (61, 301, 501, 701):
    if k >= len(files):
        continue
    prev, cur = grab(files[k - 1]), grab(files[k])
    pts = gftt(prev)
    r_cv, s_cv, _ = cv2.calcOpticalFlowPyrLK(
        prev, cur, pts.astype(np.float32).reshape(-1, 1, 2), None,
        winSize=(11, 11), maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    fl = r_cv.reshape(-1, 2) - pts.astype(np.float32)
    okc = s_cv.reshape(-1) == 1
    refu = fl[:, 0].copy(); refv = fl[:, 1].copy()
    refu[~okc] = np.nan; refv[~okc] = np.nan
    print("\n=== frame %d->%d  pts=%d  cv2 med=(%+.2f,%+.2f) ok=%d"
          % (k - 1, k, len(pts), float(np.nanmedian(refu)), float(np.nanmedian(refv)), int(okc.sum())))

    u0, v0, s0, d0 = run(prev, cur, pts)
    rep("A0 冷启 0", u0, v0, s0, refu, refv, d0)

    u3, v3, s3, d3 = run(prev, cur, pts, np.stack([np.nan_to_num(refu), np.nan_to_num(refv)], 1))
    rep("A3 初值=cv2 真值", u3, v3, s3, refu, refv, d3)

    if prev_med is not None:
        u4, v4, s4, d4 = run(prev, cur, pts, np.tile(prev_med, (len(pts), 1)))
        rep("A4 初值=上一帧中位", u4, v4, s4, refu, refv, d4)
    med = np.array([np.nanmedian(refu), np.nanmedian(refv)], np.float32)
    u5, v5, s5, d5 = run(prev, cur, pts, np.tile(med, (len(pts), 1)))
    rep("A5 初值=本帧中位(oracle)", u5, v5, s5, refu, refv, d5)

    prev_med = np.array([np.median(u0[s0 == 1]), np.median(v0[s0 == 1])], np.float32)
