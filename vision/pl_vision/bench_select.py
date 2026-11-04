#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住 cv2.goodFeaturesToTrack 的 minDistance 真实规则 + 给选择实现做基准。

Part A：合成规则点阵，看 cv2 在「间距 25 / 21 / 19 / 15」下到底留多少点
        -> 直接读出它的有效抑制半径（欧氏 r=minDist？还是夹了网格/方形？）
Part B：真实 KITTI 帧的 PL 响应图上，比较三种 select 实现的耗时
        (a) dict 网格（当前实现） (b) cv2.rectangle 掩码 (c) 候选上限封顶
"""
import os
import sys
import time
import glob

import numpy as np
import cv2

H, W, PMAX = 480, 640, 10


def log(*a):
    print(*a)
    sys.stdout.flush()


# ============ Part A：合成点阵，反推 cv2 的抑制半径 ============
log("==== A. cv2.goodFeaturesToTrack 的 minDistance 规则 ====")
for spacing in (25, 21, 19, 15):
    img = np.zeros((H, W), np.uint8)
    n_ref = 0
    for yy in range(40, 440, spacing):
        for xx in range(40, 600, spacing):
            img[yy - 1:yy + 2, xx - 1:xx + 2] = 255 if ((xx + yy) // spacing) % 2 == 0 else 210
            n_ref += 1
    c = cv2.goodFeaturesToTrack(img, maxCorners=10 ** 6, qualityLevel=0.01,
                                minDistance=20, blockSize=3, useHarrisDetector=False)
    n_cv = 0 if c is None else len(c)
    log("  间距 %2d px（>20 应全留 / <20 应被抑制）：布点 %d，cv2 留 %d -> %s"
        % (spacing, n_ref, n_cv, "全留" if n_cv == n_ref else ("全被抑制" if n_cv <= 1 else "部分")))

# 圆盘 vs 方形判别：两根轴平分处放一对「切比雪夫 ≤20 但欧氏 >20」的点
log("   -- 判圆盘/方形（切比雪夫 19、欧氏 26.9 的一对点）--")
img = np.zeros((H, W), np.uint8)
img[99:102, 99:102] = 255      # (100,100) 强
img[118:121, 118:121] = 210    # (119,119) 稍弱，欧氏 26.9 > 20，切比雪夫 19 ≤ 20
img[300:303, 300:303] = 255
img[300:303, 319:322] = 210    # 水平相距 19，欧氏 19 < 20
c = cv2.goodFeaturesToTrack(img, maxCorners=10 ** 6, qualityLevel=0.01,
                            minDistance=20, blockSize=3, useHarrisDetector=False)
pts = [] if c is None else [(int(round(p[0][0])), int(round(p[0][1]))) for p in c]
log("   cv2 留下：%s" % pts)
log("   解读：若 (119,119) 在 -> 欧氏；若不在 -> 方形/网格放大；若 (319,300) 不在 -> 抑制生效")

# ============ Part B：真实响应图上的实现基准 ============
for d in ["/home/ubuntu", "/home/ubuntu/merge_st"]:
    for f in ["_current_metadata.pkl", "global_pl_state.json"]:
        p = os.path.join(d, f)
        if os.path.exists(p):
            os.remove(p)
from pynq import Overlay, allocate  # noqa: E402

ol = Overlay("/home/ubuntu/merge_st/kv260_merged_st.bit")
st = ol.shitomasi_accel_0
rm, mm = st.register_map, st.mmio
inp = allocate(shape=(H, W), dtype=np.uint8)
out = allocate(shape=(H, W), dtype=np.uint8)
rm.img_in_1 = int(inp.device_address) & 0xFFFFFFFF
rm.img_in_2 = (int(inp.device_address) >> 32) & 0xFFFFFFFF
rm.img_out_1 = int(out.device_address) & 0xFFFFFFFF
rm.img_out_2 = (int(out.device_address) >> 32) & 0xFFFFFFFF
rm.rows, rm.cols, rm.k, rm.threshold = H, W, 0, 20


def st_run():
    mm.write(0x00, 1)
    while not ((mm.read(0x00) >> 1) & 1):
        pass


path = sorted(glob.glob("/home/ubuntu/kitti_cam/image_02/data/*.png"))[300]
img = cv2.imread(path, cv2.IMREAD_COLOR)
h2 = min(H, max(1, int(round(img.shape[0] * W / float(img.shape[1])))))
small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
canvas = np.zeros((H, W, 3), img.dtype)
top = (H - h2) // 2
canvas[top:top + h2] = small
gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
np.copyto(np.asarray(inp), gray)
inp.sync_to_device()
st_run()
out.sync_from_device()
resp = np.array(out)
resp[:PMAX, :] = 0
resp[H - PMAX:, :] = 0
resp[:, :PMAX] = 0
resp[:, W - PMAX:] = 0
NC, MD = 100, 20
log("\n==== B. 实现基准（frame300，候选 %d，NC=%d MIN_DIST=%d）====" % (int((resp > 0).sum()), NC, MD))


def order_of(r):
    ys, xs = np.nonzero(r)
    o = np.argsort(-r[ys, xs].astype(np.int32), kind='stable')
    return xs[o], ys[o]


def sel_dict(resp, nc, d):
    xs, ys = order_of(resp)
    n_cand = len(xs)
    r2 = d * d
    stride = 1024
    X, Y = xs.tolist(), ys.tolist()
    grid = {}
    pts = np.empty((nc, 2), np.int32)
    n = 0
    for i in range(n_cand):
        x, y = X[i], Y[i]
        cx, cy = x // d, y // d
        k0 = (cy - 1) * stride + cx - 1
        good = True
        for k in (k0, k0 + stride, k0 + 2 * stride):
            for kk in (k, k + 1, k + 2):
                lst = grid.get(kk)
                if lst is not None:
                    for px, py in lst:
                        dx, dy = px - x, py - y
                        if dx * dx + dy * dy < r2:
                            good = False
                            break
                    if not good:
                        break
            if not good:
                break
        if not good:
            continue
        pts[n, 0], pts[n, 1] = x, y
        n += 1
        key = cy * stride + cx
        if key in grid:
            grid[key].append((x, y))
        else:
            grid[key] = [(x, y)]
        if n >= nc:
            break
    return pts[:n]


def sel_rect(resp, nc, d):
    xs, ys = order_of(resp)
    n_cand = len(xs)
    m = np.zeros((H, W), np.uint8)
    X, Y = xs.tolist(), ys.tolist()
    pts = np.empty((nc, 2), np.int32)
    rect = cv2.rectangle
    n = 0
    for i in range(n_cand):
        x, y = X[i], Y[i]
        if m[y, x]:
            continue
        pts[n, 0], pts[n, 1] = x, y
        n += 1
        if n >= nc:
            break
        rect(m, (x - d if x > d else 0, y - d if y > d else 0), (x + d, y + d), 1, -1)
    return pts[:n]


def sel_dict_cap(resp, nc, d, cap):
    xs, ys = order_of(resp)
    n_cand = min(len(xs), cap)
    r2 = d * d
    stride = 1024
    X, Y = xs[:n_cand].tolist(), ys[:n_cand].tolist()
    grid = {}
    pts = np.empty((nc, 2), np.int32)
    n = 0
    for i in range(n_cand):
        x, y = X[i], Y[i]
        cx, cy = x // d, y // d
        k0 = (cy - 1) * stride + cx - 1
        good = True
        for k in (k0, k0 + stride, k0 + 2 * stride):
            for kk in (k, k + 1, k + 2):
                lst = grid.get(kk)
                if lst is not None:
                    for px, py in lst:
                        ddx, ddy = px - x, py - y
                        if ddx * ddx + ddy * ddy < r2:
                            good = False
                            break
                    if not good:
                        break
            if not good:
                break
        if not good:
            continue
        pts[n, 0], pts[n, 1] = x, y
        n += 1
        key = cy * stride + cx
        if key in grid:
            grid[key].append((x, y))
        else:
            grid[key] = [(x, y)]
        if n >= nc:
            break
    return pts[:n]



def sel_ba(resp, nc, d):
    """bytearray 掩码 + 预计算圆盘行跨度（欧氏 < r，与 cv2 同规则）"""
    from math import isqrt
    xs, ys = order_of(resp)
    n_cand = len(xs)
    r2 = d * d
    spans = []
    for dy in range(-d, d + 1):
        v = r2 - dy * dy - 1
        if v >= 0:
            spans.append((dy, isqrt(v)))
    X, Y = xs.tolist(), ys.tolist()
    mask = bytearray(H * W)
    ONES = b"\x01" * (2 * d + 1)
    out = []
    Wl = W
    for i in range(n_cand):
        x = X[i]
        y = Y[i]
        if mask[y * Wl + x]:
            continue
        out.append((x, y))
        if len(out) >= nc:
            break
        for dy, dxm in spans:
            yy = y + dy
            if yy < 0 or yy >= H:
                continue
            x0 = x - dxm
            if x0 < 0:
                x0 = 0
            x1 = x + dxm + 1
            if x1 > Wl:
                x1 = Wl
            o = yy * Wl + x0
            mask[o:o + (x1 - x0)] = ONES[:x1 - x0]
    return np.array(out, np.int32).reshape(-1, 2)


def bench(name, fn, *a):
    fn(*a)  # warmup
    ts = []
    for _ in range(7):
        t0 = time.perf_counter()
        pts = fn(*a)
        ts.append((time.perf_counter() - t0) * 1e3)
    ts = np.array(ts)
    log("  %-22s med=%7.3f ms  min=%7.3f  N=%d" % (name, np.median(ts), ts.min(), len(pts)))
    return np.median(ts)


n_cand = int((resp > 0).sum())
t0 = time.perf_counter()
order_of(resp)
t_sort = (time.perf_counter() - t0) * 1e3
log("  （仅 nonzero+argsort：%.3f ms）" % t_sort)
bench("bytearray+圆盘跨度", sel_ba, resp, NC, MD)
bench("dict 网格", sel_dict, resp, NC, MD)
bench("cv2.rectangle 掩码", sel_rect, resp, NC, MD)
for cap in (400, 800):
    bench("dict + 候选封顶 %d" % cap, sel_dict_cap, resp, NC, MD, cap)
log("  loadavg: %s" % open("/proc/loadavg").read().strip())
log("=== DONE ===")
