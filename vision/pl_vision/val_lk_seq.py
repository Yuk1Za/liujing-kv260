#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""序列级 A/B：两种喂法，各跑 24 帧，与 cv2 3 层 LK 的中位流对拍。

R 模式（节点现状）：每帧在 prev 上重新检测 -> 冷启 0 -> 跟踪
P 模式（VINS 语义）：沿用上一帧跟踪到的点 -> 用每点上一帧位移热启 -> 跟踪 -> 只补新点
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
    if n == 0:
        return np.zeros(0, np.float32), np.zeros(0, np.float32), np.zeros(0, np.uint8), 0.0
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


def gftt(g, n=NC):
    p = cv2.goodFeaturesToTrack(g, n, QL, MIN_DIST, mask=MASK, blockSize=3)
    return np.round(p.reshape(-1, 2)).astype(np.int32) if p is not None else np.zeros((0, 2), np.int32)


def ref_flow(prev, cur, pts):
    if len(pts) == 0:
        return np.zeros(0), np.zeros(0)
    r, s, _ = cv2.calcOpticalFlowPyrLK(
        prev, cur, pts.astype(np.float32).reshape(-1, 1, 2), None,
        winSize=(11, 11), maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    fl = r.reshape(-1, 2) - pts.astype(np.float32)
    ok = s.reshape(-1) == 1
    u, v = fl[:, 0].copy(), fl[:, 1].copy()
    u[~ok] = np.nan; v[~ok] = np.nan
    return u, v


def replenish(cur_pts, cur_img, want, min_d):
    """用当前帧新检测点把点数补齐（新点与已有点保持 min_d 距离）"""
    if len(cur_pts) >= want:
        return cur_pts
    cand = gftt(cur_img, want * 3)
    keep = []
    for x, y in cand:
        if len(cur_pts) + len(keep) >= want:
            break
        allp = np.vstack([cur_pts, np.array(keep, np.int32).reshape(-1, 2)]) if keep else cur_pts
        if len(allp) and (((allp[:, 0] - x) ** 2 + (allp[:, 1] - y) ** 2) < min_d * min_d).any():
            continue
        keep.append((x, y))
    if not keep:
        return cur_pts
    return np.vstack([cur_pts, np.array(keep, np.int32)])


def run_seq(k0, nfr, label):
    files = sorted(glob.glob("/home/ubuntu/kitti_cam/image_02/data/*.png"))
    imgs = {k: grab(files[k]) for k in range(k0 - 1, k0 + nfr)}
    rows = []
    # ---- R 模式 ----
    tR = 0.0
    for k in range(k0, k0 + nfr):
        prev, cur = imgs[k - 1], imgs[k]
        pts = gftt(prev)
        ru, rv = ref_flow(prev, cur, pts)
        u, v, st, dt = run(prev, cur, pts, None)
        ok = (st == 1) & np.isfinite(ru)
        rows.append(("R", k, float(np.median(u[ok])), float(np.nanmedian(ru)), int(ok.sum()), dt))
        tR += dt
    # ---- P 模式 ----
    tP = 0.0
    prev_pts, prev_fl = gftt(imgs[k0 - 1]), None
    for k in range(k0, k0 + nfr):
        prev, cur = imgs[k - 1], imgs[k]
        ru, rv = ref_flow(prev, cur, prev_pts)
        u, v, st, dt = run(prev, cur, prev_pts, prev_fl)
        ok = st == 1
        cur_pts = prev_pts[ok] + np.stack([u[ok], v[ok]], 1).round().astype(np.int32)
        cur_fl = np.stack([u[ok], v[ok]], 1).astype(np.float32)
        n_tracked = len(cur_pts)
        # 补齐（模拟 VINS：跟踪为主，掉了才补）
        cur_pts2 = replenish(cur_pts, cur, NC, MIN_DIST)
        n_new = len(cur_pts2) - n_tracked
        if n_new:
            fill = np.median(cur_fl, 0) if len(cur_fl) else np.zeros(2, np.float32)
            cur_fl = np.vstack([cur_fl, np.tile(fill.astype(np.float32), (n_new, 1))])
        okr = np.isfinite(ru)
        # P 模式的中位数只能在与 cv2 同一点集上比，用跟踪存活的那批
        rows.append(("P", k, float(np.median(u[ok])), float(np.nanmedian(ru[ok])), n_tracked, dt))
        tP += dt
        prev_pts, prev_fl = cur_pts2, cur_fl
    print("\n=== %s  (frame %d..%d) ===" % (label, k0, k0 + nfr - 1))
    print(" %-4s %5s | %8s %8s %8s | %7s %5s" % ("mode", "k", "PL med_u", "cv2 med_u", "比值", "ok", "ms"))
    errR = errP = 0.0
    for m, k, pu, cu, nok, dt in rows:
        ratio = pu / cu if abs(cu) > 1e-3 else float('nan')
        print(" %-4s %5d | %+8.2f %+8.2f %8.2f | %7d %5.2f" % (m, k, pu, cu, ratio, nok, dt * 1e3))
        if m == "R":
            errR += abs(pu - cu)
        else:
            errP += abs(pu - cu)
    print("  ⇒ 平均 |PL-cv2|：R 模式 %.3f px | P 模式 %.3f px  （每帧平均耗时 R %.2f / P %.2f ms）"
          % (errR / nfr, errP / nfr, tR / nfr * 1e3, tP / nfr * 1e3))
    sys.stdout.flush()


run_seq(63, 16, "转弯段（cv2 中位流较大）")
run_seq(700, 16, "直行段")
