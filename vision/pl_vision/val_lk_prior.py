#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""初值策略 A/B（每帧仍重新检测，只换 flow_io 初值）：

  C0 冷启 0
  C1 常量先验 = 上一帧有效点 (u,v) 的中位数
  C2 仿射先验 u,v ~ a + b*x + c*y，系数由上一帧流场最小二乘拟合
     （旋转主导时流场在图像上是近线性的，比常量准得多）

判据：PL 中位流 / cv2 中位流 的比值越接近 1 越好；同时看耗时。
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
mm = ol.slk_accel_0.mmio
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


def cvref(prev, cur, pts):
    r, s, _ = cv2.calcOpticalFlowPyrLK(
        prev, cur, pts.astype(np.float32).reshape(-1, 1, 2), None,
        winSize=(11, 11), maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    fl = r.reshape(-1, 2) - pts.astype(np.float32)
    return fl[:, 0], fl[:, 1], s.reshape(-1) == 1


def affine_init(px, py, u, v):
    A = np.stack([np.ones(len(px)), px, py], 1).astype(np.float64)
    cu = np.linalg.lstsq(A, u, rcond=None)[0]
    cv_ = np.linalg.lstsq(A, v, rcond=None)[0]
    return lambda X, Y: (cu[0] + cu[1] * X + cu[2] * Y, cv_[0] + cv_[1] * X + cv_[2] * Y)


def seq(k0, nfr, label):
    files = sorted(glob.glob("/home/ubuntu/kitti_cam/image_02/data/*.png"))
    imgs = {k: grab(files[k]) for k in range(k0 - 1, k0 + nfr)}
    prev_state = None       # (pts, u, v) 上一帧结果
    print("\n=== %s (frame %d..%d) ===" % (label, k0, k0 + nfr - 1))
    print(" %5s | %8s %8s %8s %8s | %s" % ("k", "cv2", "C0冷启", "C1常量", "C2仿射", "最优"))
    e = {c: 0.0 for c in ("C0", "C1", "C2")}
    n = 0
    for k in range(k0, k0 + nfr):
        prev, cur = imgs[k - 1], imgs[k]
        pts = gftt(prev)
        cu, cv_, okc = cvref(prev, cur, pts)
        ref = float(np.median(cu[okc]))

        u0, v0, s0, _ = run(prev, cur, pts)
        m0 = float(np.median(u0[s0 == 1]))

        if prev_state is None:
            m1 = m2 = float('nan')
        else:
            pp, pu, pv = prev_state
            ok = np.isfinite(pu)
            mu, mv = float(np.median(pu[ok])), float(np.median(pv[ok]))
            u1, v1, s1, _ = run(prev, cur, pts, np.full((len(pts), 2), (mu, mv), np.float32))
            m1 = float(np.median(u1[s1 == 1]))
            f = affine_init(pp[ok, 0].astype(np.float64), pp[ok, 1].astype(np.float64),
                            pu[ok].astype(np.float64), pv[ok].astype(np.float64))
            iu, iv = f(pts[:, 0].astype(np.float64), pts[:, 1].astype(np.float64))
            u2, v2, s2, _ = run(prev, cur, pts, np.stack([iu, iv], 1).astype(np.float32))
            m2 = float(np.median(u2[s2 == 1]))
            e["C1"] += abs(m1 - ref)
            e["C2"] += abs(m2 - ref)
        e["C0"] += abs(m0 - ref)
        n += 1
        cands = [("C0", abs(m0 - ref))]
        if m1 == m1:
            cands.append(("C1", abs(m1 - ref)))
        if m2 == m2:
            cands.append(("C2", abs(m2 - ref)))
        print(" %5d | %+8.2f %+8.2f %+8.2f %+8.2f | %s"
              % (k, ref, m0, m1, m2, min(cands, key=lambda t: t[1])[0]))
        # 存本帧（冷启）结果作为下一帧先验，保证可部署
        prev_state = (pts, np.where(s0 == 1, u0, np.nan).astype(np.float32),
                      np.where(s0 == 1, v0, np.nan).astype(np.float32))
        sys.stdout.flush()
    n1 = max(1, n - 1)
    print("  ⇒ 平均 |Δ中位流|：C0 %.3f px | C1 %.3f px | C2 %.3f px（C1/C2 从第 2 帧起算）"
          % (e["C0"] / n, e["C1"] / n1, e["C2"] / n1))


seq(63, 16, "转弯段")
seq(295, 12, "另一转弯段")
seq(700, 10, "直行段")
