#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LK 基准：同一块 KV260 上，CPU(cv2) vs PL(我们的稀疏 LK IP)

目的：量化「把 Coco-LIC 的 LK 搬到 PL」值不值。
 - CPU-A：Coco-LIC 实际配置  winSize=(21,21), 3 层, 10 iter / EPS 0.05, GET_MIN_EIGENVALS
 - CPU-B：与我们 IP 同参数  winSize=(11,11), 2 层, 20 iter / EPS 0.01
 - PL   ：我们的稀疏 LK IP（WH=5, SEARCH=4 => 约 ±9 px 搜索窗, 2 层, MAXIT=20）
图像与点集完全一致，只比耗时；算法参数差异单独列出，不混为一谈。
"""
import os
import time
import numpy as np
import cv2
from pynq import Overlay, allocate

F_AP, F_IN, F_TH, F_OUT, F_R, F_C = 0x00, 0x10, 0x1c, 0x24, 0x30, 0x38
S_AP, S_PREV, S_CUR, S_COR, S_FLOW, S_ST = 0x00, 0x10, 0x1c, 0x28, 0x34, 0x40
S_N, S_ROWS, S_COLS, S_ITER = 0x4c, 0x54, 0x5c, 0x64
W, H, PMAX, Q, MAXIT = 640, 480, 10, 6, 20
NC = 300                      # Coco-LIC 的 m_maximum_vio_tracked_pts
PAIRS = [(95, 96), (127, 128)]
BIT = '/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/share/lo_sam/bitstreams/feature_extraction.bit'
IMGDIR = '/home/ubuntu/kitti_cam/image_02/data'
REP = 20


def wr64(mm, a, v):
    mm.write(a, v & 0xFFFFFFFF)
    mm.write(a + 4, (v >> 32) & 0xFFFFFFFF)


def pack(img):
    r, c = img.shape
    return img.reshape(r, c // 4, 4).copy().view(np.uint32).reshape(r, c // 4)


def grab(i):
    img = cv2.imread(os.path.join(IMGDIR, '%010d.png' % i), cv2.IMREAD_COLOR)
    h0, w0 = img.shape[:2]
    h2 = min(H, max(1, int(round(h0 * W / float(w0)))))
    small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((H, W, 3), np.uint8)
    top = (H - h2) // 2
    canvas[top:top + h2] = small
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)


def med(fn, rep=REP):
    fn()
    ts = []
    for _ in range(rep):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1e3)
    return float(np.median(ts)), float(np.min(ts))


ol = Overlay(BIT, download=True)
print('PL IP:', sorted(ol.ip_dict))
sm = ol.slk_accel_0.mmio
pb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cor_b = allocate((NC,), dtype=np.uint32, cacheable=1)
fl_b = allocate((NC,), dtype=np.uint32, cacheable=1)
st_b = allocate((NC,), dtype=np.uint32, cacheable=1)
wr64(sm, S_PREV, pb.device_address); wr64(sm, S_CUR, cb.device_address)
wr64(sm, S_COR, cor_b.device_address); wr64(sm, S_FLOW, fl_b.device_address)
wr64(sm, S_ST, st_b.device_address)
sm.write(S_ROWS, H); sm.write(S_COLS, W); sm.write(S_ITER, MAXIT)
sm.write(S_N, NC)

print('cv2.getNumThreads() =', cv2.getNumThreads())
for ka, kb in PAIRS:
    prev = grab(ka)
    cur = grab(kb)
    # 300 个点：在有效行区内均匀撒，带抖动（模拟 LiDAR 投影点分布）
    rng = np.random.default_rng(0)
    y0, y1 = 143 + PMAX + 2, 336 - PMAX - 2
    xs = rng.uniform(PMAX + 2, W - PMAX - 3, NC)
    ys = rng.uniform(y0, y1, NC)
    pts = np.stack([xs, ys], 1).astype(np.float32)
    pts_i = pts.astype(np.int32)

    print('\n===== frames %d -> %d   N=%d  image %dx%d =====' % (ka, kb, NC, W, H))

    def cpu(win, maxlvl, crit, nthr):
        cv2.setNumThreads(nthr)
        cv2.calcOpticalFlowPyrLK(prev, cur, pts, None, winSize=win, maxLevel=maxlvl,
                                 criteria=crit, flags=cv2.OPTFLOW_USE_INITIAL_FLOW
                                 if False else 0)

    m, _ = med(lambda: cpu((21, 21), 2, (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 10, 0.05), 1))
    print('CPU-A  Coco-LIC 配置 (21x21, 3 层, 10 iter)  单线程 : %.2f ms' % m)
    m4, _ = med(lambda: cpu((21, 21), 2, (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 10, 0.05), 4))
    print('CPU-A  同上                                  4 线程 : %.2f ms' % m4)
    m, _ = med(lambda: cpu((11, 11), 1, (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 20, 0.01), 1))
    print('CPU-B  与 PL 同参数 (11x11, 2 层, 20 iter)   单线程 : %.2f ms' % m)
    m4, _ = med(lambda: cpu((11, 11), 1, (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 20, 0.01), 4))
    print('CPU-B  同上                                  4 线程 : %.2f ms' % m4)
    cv2.setNumThreads(4)

    # ---- PL：整帧成本 = 两张图上传 + IP 执行 ----
    def pl_run():
        pb[:] = pack(prev); pb.sync_to_device()
        cb[:] = pack(cur); cb.sync_to_device()
        cor_b[:NC] = (pts_i[:, 1].astype(np.uint32) << 16) | (pts_i[:, 0].astype(np.uint32) & 0xFFFF)
        cor_b.sync_to_device()
        sm.write(S_AP, 1)
        while not ((sm.read(S_AP) >> 1) & 1):
            time.sleep(0.0002)

    t_poll = med(lambda: sm.write(S_AP, 1) or [time.sleep(0.0002) for _ in range(0)] and
                 (while_wait(sm) if False else None), rep=1) if False else None
    # 分开测：仅 IP 执行（图与点已在 device 上）
    pb[:] = pack(prev); pb.sync_to_device()
    cb[:] = pack(cur); cb.sync_to_device()
    cor_b[:NC] = (pts_i[:, 1].astype(np.uint32) << 16) | (pts_i[:, 0].astype(np.uint32) & 0xFFFF)
    cor_b.sync_to_device()

    def ip_only():
        sm.write(S_AP, 1)
        while not ((sm.read(S_AP) >> 1) & 1):
            time.sleep(0.0002)

    t_ip, _ = med(ip_only)
    t_all, _ = med(pl_run)
    fl_b.sync_from_device(); st_b.sync_from_device()
    raw = np.array(fl_b).view(np.int16).reshape(NC, 2)
    u = raw[:, 1].astype(np.float32) / (1 << Q)
    st = np.array(st_b)
    print('PL     仅 IP 执行                                  : %.2f ms  (%.1f us/点)'
          % (t_ip, t_ip * 1e3 / NC))
    print('PL     整帧 (2 张图上传 + IP)                      : %.2f ms' % t_all)
    print('       status=1 比例 %.0f%%   中位 |u| %.2f px'
          % (100.0 * (st == 1).sum() / NC, float(np.median(np.abs(u[st == 1]))) if (st == 1).any() else float('nan')))
