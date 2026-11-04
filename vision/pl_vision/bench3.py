#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""选择实现第三轮：把 dict.get 换成「扁平 cell 列表」，并核对它与 dict 版逐点一致。

板端实测（A53）：dict.get ≈1.5 µs、numpy 标量读 ≈5 µs、bytearray 行切片写 ≈6 µs，
纯 Python 空循环 ≈1 µs/次 —— 所以优化方向只有一个：**减少每次候选的 Python 操作数**。
"""
import os
import sys
import time
import glob
from math import isqrt

import numpy as np
import cv2

H, W, PMAX = 480, 640, 10


def log(*a):
    print(*a)
    sys.stdout.flush()


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


def pl_resp(idx):
    path = sorted(glob.glob("/home/ubuntu/kitti_cam/image_02/data/*.png"))[idx]
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    h2 = min(H, max(1, int(round(img.shape[0] * W / float(img.shape[1])))))
    small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((H, W, 3), img.dtype)
    top = (H - h2) // 2
    canvas[top:top + h2] = small
    gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    np.copyto(np.asarray(inp), gray)
    inp.sync_to_device()
    mm.write(0x00, 1)
    while not ((mm.read(0x00) >> 1) & 1):
        pass
    out.sync_from_device()
    r = np.array(out)
    r[:PMAX, :] = 0
    r[H - PMAX:, :] = 0
    r[:, :PMAX] = 0
    r[:, W - PMAX:] = 0
    return r


def order_of(r):
    ys, xs = np.nonzero(r)
    o = np.argsort(-r[ys, xs].astype(np.int32), kind="stable")
    return xs[o].tolist(), ys[o].tolist()


def sel_dict(r, nc, d):
    X, Y = order_of(r)
    n_cand = len(X)
    r2 = d * d
    stride = 1024
    grid = {}
    pts = []
    for i in range(n_cand):
        x = X[i]
        y = Y[i]
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
        pts.append((x, y))
        if len(pts) >= nc:
            break
        key = cy * stride + cx
        lst = grid.get(key)
        if lst is None:
            grid[key] = [(x, y)]
        else:
            lst.append((x, y))
    return np.array(pts, np.int32).reshape(-1, 2)


def sel_grill(r, nc, d):
    """扁平 cell 列表（None 占位），查询用 9 次 list 索引"""
    X, Y = order_of(r)
    n_cand = len(X)
    r2 = d * d
    stride = 1024
    grow = W // d + 3
    gh = H // d + 3
    cells = [None] * (stride * gh)
    base = stride + 1              # 让 (cy-1)*stride+(cx-1) 从 0 起算
    pts = []
    for i in range(n_cand):
        x = X[i]
        y = Y[i]
        cx, cy = x // d, y // d
        k0 = cy * stride + cx
        good = True
        for kk in (k0 - stride - 1, k0 - stride, k0 - stride + 1,
                   k0 - 1, k0, k0 + 1,
                   k0 + stride - 1, k0 + stride, k0 + stride + 1):
            lst = cells[kk]
            if lst is not None:
                for px, py in lst:
                    dx = px - x
                    dy = py - y
                    if dx * dx + dy * dy < r2:
                        good = False
                        break
                if not good:
                    break
        if not good:
            continue
        pts.append((x, y))
        if len(pts) >= nc:
            break
        if cells[k0] is None:
            cells[k0] = [(x, y)]
        else:
            cells[k0].append((x, y))
    return np.array(pts, np.int32).reshape(-1, 2)



def sel_vec(r, nc, d):
    """向量化贪心：每轮取第一个未屏蔽候选，整数组一次算出被它屏蔽的集合"""
    pos = cv2.findNonZero(r)
    if pos is None:
        return np.zeros((0, 2), np.int32)
    pos = pos.reshape(-1, 2)
    order = np.argsort(-r[pos[:, 1], pos[:, 0]].astype(np.int32), kind="stable")
    P = pos[order]
    PX = P[:, 0].astype(np.int32)
    PY = P[:, 1].astype(np.int32)
    n = len(PX)
    r2 = d * d
    blocked = np.zeros(n, bool)
    kept = []
    for _ in range(nc):
        i = int(np.argmin(blocked))
        if blocked[i]:
            break
        kept.append(i)
        dx = PX - PX[i]
        dy = PY - PY[i]
        blocked |= (dx * dx + dy * dy) < r2
    return P[kept] if kept else np.zeros((0, 2), np.int32)


def sel_rect(r, nc, d):
    X, Y = order_of(r)
    n_cand = len(X)
    m = np.zeros((H, W), np.uint8)
    rect = cv2.rectangle
    pts = []
    for i in range(n_cand):
        x = X[i]
        y = Y[i]
        if m[y, x]:
            continue
        pts.append((x, y))
        if len(pts) >= nc:
            break
        rect(m, (x - d if x > d else 0, y - d if y > d else 0), (x + d, y + d), 1, -1)
    return np.array(pts, np.int32).reshape(-1, 2)


def bench(name, fn, r, nc, d, n=7):
    fn(r, nc, d)
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        pts = fn(r, nc, d)
        ts.append((time.perf_counter() - t0) * 1e3)
    log("  %-26s med=%7.3f ms  min=%7.3f  N=%d" % (name, float(np.median(ts)), min(ts), len(pts)))
    return np.median(ts), pts


NC, MD = 100, 20
log("loadavg %s" % open("/proc/loadavg").read().strip())
for fidx in (0, 300, 700):
    r = pl_resp(fidx)
    log("\n=== frame %d：候选 %d ===" % (fidx, int((r > 0).sum())))
    bench("向量化贪心", sel_vec, r, NC, MD)
    bench("扁平 cell 列表（欧氏）", sel_grill, r, NC, MD)
    bench("dict 网格（欧氏）", sel_dict, r, NC, MD)
    bench("cv2.rectangle（方形）", sel_rect, r, NC, MD)
    a = sel_grill(r, NC, MD)
    b = sel_dict(r, NC, MD)
    same = (a.shape == b.shape) and bool(np.array_equal(a, b))
    log("  扁平列表 vs dict 逐点一致: %s（%d vs %d）" % (same, len(a), len(b)))
    v = sel_vec(r, NC, MD)
    same2 = (a.shape == v.shape) and bool(np.array_equal(a, v))
    log("  向量化 vs 扁平列表 逐点一致: %s（%d vs %d）" % (same2, len(a), len(v)))
log("=== DONE ===")
