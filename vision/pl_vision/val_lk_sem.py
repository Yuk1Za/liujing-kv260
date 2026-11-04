#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""稀疏 LK 语义 + 标定：corners 归属哪帧、符号、绝对幅度（合成已知位移）。

A) corners = prev 检测点（内核设计语义）
B) corners = cur  检测点（节点当前错误用法）
S) 合成：把 prev 平移 (+3,-2) 当作 cur，喂 prev 的点，看内核给出的绝对值
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
wr64(mm, S_PREV, pb.device_address)
wr64(mm, S_CUR, cb.device_address)
wr64(mm, S_COR, cor_b.device_address)
wr64(mm, S_FLOW, fl_b.device_address)
wr64(mm, S_ST, st_b.device_address)
mm.write(S_ROWS, H)
mm.write(S_COLS, W)
mm.write(S_ITER, MAXIT)


def run(prev, cur, pts, f0=None, iters=None):
    if iters is not None:
        mm.write(S_ITER, iters)
    np.copyto(np.asarray(pb), pack(prev)); pb.sync_to_device()
    np.copyto(np.asarray(cb), pack(cur)); cb.sync_to_device()
    n = len(pts)
    cor = (pts[:, 1].astype(np.uint32) << 16) | (pts[:, 0].astype(np.uint32) & 0xFFFF)
    cor_b[:n] = cor
    if f0 is None:
        fl_b[:n] = 0
    else:
        fu = np.clip(np.round(f0[:, 0] * (1 << Q)), -32768, 32767).astype(np.int32)
        fv = np.clip(np.round(f0[:, 1] * (1 << Q)), -32768, 32767).astype(np.int32)
        fl_b[:n] = ((fu.astype(np.uint32) & 0xFFFF) << 16) | (fv.astype(np.uint32) & 0xFFFF)
    cor_b.sync_to_device()
    fl_b.sync_to_device()
    mm.write(S_N, n)
    t0 = time.perf_counter()
    mm.write(S_AP, 1)
    while not ((mm.read(S_AP) >> 1) & 1):
        pass
    dt = time.perf_counter() - t0
    fl_b.sync_from_device()
    st_b.sync_from_device()
    raw = np.array(fl_b).view(np.int16).reshape(NC, 2)[:n]
    u = raw[:, 1].astype(np.float32) / (1 << Q)
    v = raw[:, 0].astype(np.float32) / (1 << Q)
    st = np.array(st_b)[:n].astype(np.uint8)
    if iters is not None:
        mm.write(S_ITER, MAXIT)
    return u, v, st, dt


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


def stat(tag, u, v, st, refu=None, refv=None, dt=None):
    ok = st == 1
    if refu is not None:
        ok &= np.isfinite(refu) & np.isfinite(refv)
    su = "med=(%+7.2f,%+7.2f)" % (float(np.median(u[ok])) if ok.any() else float('nan'),
                                  float(np.median(v[ok])) if ok.any() else float('nan'))
    line = "  %-20s ok %3d/%-3d %s" % (tag, int(ok.sum()), len(st), su)
    if refu is not None and ok.sum() > 5:
        du, dv = u[ok] - refu[ok], v[ok] - refv[ok]
        r = np.corrcoef(u[ok], refu[ok])[0, 1]
        # 斜率：PL = a*cv2 + b（最小二乘），a 明显 <1 就是欠收敛
        A = np.stack([refu[ok], np.ones(ok.sum())], 1)
        a = np.linalg.lstsq(A, u[ok], rcond=None)[0][0]
        line += " | ref med=%+7.2f | err(%+.2f,%+.2f) p90=%.2f | r=%+.3f 斜率=%.2f" % (
            float(np.median(refu[ok])), float(np.median(du)), float(np.median(dv)),
            float(np.percentile(np.hypot(du, dv), 90)), r, a)
    if dt is not None:
        line += " | %.2f ms" % (dt * 1e3)
    print(line)
    sys.stdout.flush()


# ---------------- S) 合成整数位移：验内核绝对标定 ----------------
files = sorted(glob.glob("/home/ubuntu/kitti_cam/image_02/data/*.png"))
print("frames:", len(files))
g = grab(files[700])
pg = gftt(g)
print("\n=== S) 合成平移（真值已知）===")
for (sx, sy) in ((1, 0), (3, -2), (6, 4), (8, 0)):
    # cur 由 prev 平移得到：内容向右下移动 -> 光流 = (+sx,+sy)，用 warpAffine 反向采样
    M = np.float32([[1, 0, -sx], [0, 1, -sy]])
    cur = cv2.warpAffine(g, M, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    u, v, st, dt = run(g, cur, pg)
    ok = st == 1
    print("  真值 (%+d,%+d): PL med=(%+6.2f,%+6.2f)  mean=(%+6.2f,%+6.2f)  误差<0.5 占比 %.0f%%  ok=%d/%d  %.2f ms"
          % (sx, sy, float(np.median(u[ok])), float(np.median(v[ok])),
             float(u[ok].mean()), float(v[ok].mean()),
             100.0 * float(((np.abs(u[ok] - sx) < 0.5) & (np.abs(v[ok] - sy) < 0.5)).mean()),
             int(ok.sum()), len(st), dt * 1e3))
    sys.stdout.flush()
    # cv2 同一对图（单层窗口 11，模拟 IP 的窗）
    r_cv, s_cv, _ = cv2.calcOpticalFlowPyrLK(
        g, cur, pg.astype(np.float32).reshape(-1, 1, 2), None,
        winSize=(11, 11), maxLevel=0,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01))
    fl = r_cv.reshape(-1, 2) - pg.astype(np.float32)
    okc = s_cv.reshape(-1) == 1
    print("            cv2 单层:     med=(%+6.2f,%+6.2f)  ok=%d/%d"
          % (float(np.median(fl[okc, 0])), float(np.median(fl[okc, 1])), int(okc.sum()), len(pg)))

# ---------------- 真实连续帧 ----------------
for k in (61, 301, 701):
    if k >= len(files):
        continue
    prev, cur = grab(files[k - 1]), grab(files[k])
    p_prev, p_cur = gftt(prev), gftt(cur)
    print("\n=== frame %d->%d   prev_pts=%d cur_pts=%d" % (k - 1, k, len(p_prev), len(p_cur)))

    r_cv, s_cv, _ = cv2.calcOpticalFlowPyrLK(
        prev, cur, p_prev.astype(np.float32).reshape(-1, 1, 2), None,
        winSize=(11, 11), maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    refu = (r_cv.reshape(-1, 2) - p_prev.astype(np.float32))[:, 0].copy()
    refv = (r_cv.reshape(-1, 2) - p_prev.astype(np.float32))[:, 1].copy()
    refu[s_cv.reshape(-1) == 0] = np.nan
    refv[s_cv.reshape(-1) == 0] = np.nan
    print("  cv2 3层(prev->cur) med=(%+.2f,%+.2f) ok=%d"
          % (float(np.nanmedian(refu)), float(np.nanmedian(refv)), int(s_cv.sum())))

    ua, va, sa, dta = run(prev, cur, p_prev)
    stat("A prev-corners 冷启", ua, va, sa, refu, refv, dta)
    ua2, va2, sa2, dta2 = run(prev, cur, p_prev, np.stack([ua, va], 1))
    stat("A2 prev-corners 热启", ua2, va2, sa2, refu, refv, dta2)
    for it in (5, 50):
        ui, vi, si, dti = run(prev, cur, p_prev, None, iters=it)
        stat("A it=%d" % it, ui, vi, si, refu, refv, dti)

    ub, vb, sb, dtb = run(prev, cur, p_cur)
    r2, s2, _ = cv2.calcOpticalFlowPyrLK(
        prev, cur, p_cur.astype(np.float32).reshape(-1, 1, 2), None,
        winSize=(11, 11), maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    b2 = r2.reshape(-1, 2) - p_cur.astype(np.float32)
    refbu, refbv = b2[:, 0].copy(), b2[:, 1].copy()
    refbu[s2.reshape(-1) == 0] = np.nan
    refbv[s2.reshape(-1) == 0] = np.nan
    stat("B cur-corners", ub, vb, sb, refbu, refbv, dtb)
    print("      cv2 同喂法 med=%+.2f  => PL 与 cv2 同号？ %s"
          % (float(np.nanmedian(refbu)),
             "是" if np.sign(np.nanmedian(refbu)) == np.sign(np.median(ub)) else "**否**"))
