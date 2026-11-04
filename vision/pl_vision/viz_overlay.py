#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可视化：把 PL 的 FAST + 稀疏 LK 在真实 KITTI 帧上的结果画出来

对每个帧对 (k, k+1)：背景 = 第 k 帧灰度图；绿圈 = FAST 角点；
绿箭头 = 该点被跟踪到的位移（在搜索窗内）；红箭头 = |u| 打到 ±9 px 搜索窗边界（饱和，值不可信）；
灰圈 = status==0（跟踪后出界，flow 缓冲区里是残留值，不可用）。

结论直观看点：饱和率一高（急转弯），红箭头就一大片 —— 这正是视觉偏航跟不上的原因。
"""
import os
import sys
import time
import numpy as np
import cv2
from pynq import Overlay, allocate

F_AP, F_IN, F_TH, F_OUT, F_R, F_C = 0x00, 0x10, 0x1c, 0x24, 0x30, 0x38
S_AP, S_PREV, S_CUR, S_COR, S_FLOW, S_ST = 0x00, 0x10, 0x1c, 0x28, 0x34, 0x40
S_N, S_ROWS, S_COLS, S_ITER = 0x4c, 0x54, 0x5c, 0x64
W, H = 640, 480
WH, SEARCH = 5, 4
PMAX = WH + SEARCH + 1
Q = 6
MAXIT = 20
NC = 100          # 与 pl_vision_node.py 默认一致
SAT = 8.0         # 饱和阈值 px
BIT = '/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/share/lo_sam/bitstreams/feature_extraction.bit'
IMGDIR = '/home/ubuntu/kitti_cam/image_02/data'
CAP = 30


def wr64(mm, a, v):
    mm.write(a, v & 0xFFFFFFFF)
    mm.write(a + 4, (v >> 32) & 0xFFFFFFFF)


def pack(img):
    r, c = img.shape
    return img.reshape(r, c // 4, 4).copy().view(np.uint32).reshape(r, c // 4)


def grab(i):
    p = os.path.join(IMGDIR, '%010d.png' % i)
    img = cv2.imread(p, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit('读图失败 ' + p)
    h0, w0 = img.shape[:2]
    h2 = min(H, max(1, int(round(h0 * W / float(w0)))))
    small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((H, W, 3), np.uint8)
    top = (H - h2) // 2
    canvas[top:top + h2] = small
    return canvas, cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY), (top, h2)


def main():
    pairs = [(55, 56), (95, 96), (127, 128)]
    ol = Overlay(BIT, download=True)
    print('PL IP:', sorted(ol.ip_dict))
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
    fm.write(F_TH, 20)
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

    panels = []
    for (ka, kb) in pairs:
        prev_col, prev, _ = grab(ka)
        _, cur, _ = grab(kb)

        inp[:] = prev
        inp.sync_to_device()
        out.fill(0)
        out.sync_to_device()
        fm.write(F_AP, 1)
        while not ((fm.read(F_AP) >> 1) & 1):
            time.sleep(0.0002)
        out.sync_from_device()
        ys, xs = np.nonzero(np.array(out))
        n_all = len(xs)
        m = (xs >= PMAX) & (ys >= PMAX) & (xs < W - PMAX) & (ys < H - PMAX)
        sel = np.stack([xs[m], ys[m]], 1).astype(np.int32)
        if len(sel) > NC:
            sel = sel[np.linspace(0, len(sel) - 1, NC).astype(np.int64)]
        pts = sel
        N = len(pts)

        pb[:] = pack(prev)
        pb.sync_to_device()
        cb[:] = pack(cur)
        cb.sync_to_device()
        cor_b[:N] = (pts[:, 1].astype(np.uint32) << 16) | (pts[:, 0].astype(np.uint32) & 0xFFFF)
        cor_b.sync_to_device()
        sm.write(S_N, N)
        sm.write(S_AP, 1)
        while not ((sm.read(S_AP) >> 1) & 1):
            time.sleep(0.0002)
        fl_b.sync_from_device()
        st_b.sync_from_device()
        raw = np.array(fl_b).view(np.int16).reshape(NC, 2)
        u = raw[:N, 1].astype(np.float32) / (1 << Q)
        v = raw[:N, 0].astype(np.float32) / (1 << Q)
        st = np.array(st_b)[:N].astype(np.int32)

        canvas = prev_col.copy()
        cv2.rectangle(canvas, (PMAX, PMAX), (W - PMAX - 1, H - PMAX - 1), (70, 70, 70), 1)
        ok = st == 1
        sat = ok & (np.abs(u) >= SAT)
        good = ok & ~sat
        nsat = int(sat.sum())
        for i in np.nonzero(~ok)[0]:
            cv2.circle(canvas, (int(pts[i, 0]), int(pts[i, 1])), 2, (130, 130, 130), -1)
        for i in np.nonzero(good)[0]:
            p0 = (int(pts[i, 0]), int(pts[i, 1]))
            p1 = (int(round(pts[i, 0] + u[i])), int(round(pts[i, 1] + v[i])))
            cv2.arrowedLine(canvas, p0, p1, (140, 220, 60), 1, tipLength=0.32)
            cv2.circle(canvas, p0, 2, (60, 200, 40), -1)
        for i in np.nonzero(sat)[0]:
            p0 = (int(pts[i, 0]), int(pts[i, 1]))
            p1 = (int(round(pts[i, 0] + u[i])), int(round(pts[i, 1] + v[i])))
            cv2.arrowedLine(canvas, p0, p1, (60, 60, 235), 1, tipLength=0.32)
            cv2.circle(canvas, p0, 2, (40, 40, 230), -1)

        mu = float(np.median(u[ok])) if ok.any() else float('nan')
        dv = -mu / 371.8 * 57.2958
        bar = np.zeros((CAP, W, 3), np.uint8)
        txt = ('frames %d->%d  corners %d (FAST %d)  valid %d  saturated %d (%.0f%%)  '
               'med_u %+.2f px  d_yaw %+.2f deg'
               % (ka, kb, N, n_all, int(ok.sum()), nsat,
                  100.0 * nsat / max(1, int(ok.sum())), mu, dv))
        cv2.putText(bar, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (230, 230, 230), 1, cv2.LINE_AA)
        panels.append(np.vstack([bar, canvas]))
        print(txt)

    legend = np.zeros((CAP, W, 3), np.uint8)
    cv2.putText(legend, 'green arrow = flow inside +/-9px search window   |   '
                        'RED = saturated at window edge (unreliable)   |   grey dot = status 0 (out of frame)',
                (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)
    img = np.vstack([legend] + panels)
    outp = '/home/ubuntu/pl_vision/viz/flow_overlay.png'
    cv2.imwrite(outp, img)
    print('WROTE', outp, img.shape)


main()
