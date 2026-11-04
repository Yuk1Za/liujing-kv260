#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一图看全：PL 的 FAST + 稀疏 LK 在真实 KITTI 帧上的结果 + |u| 分布

上：3 个帧对的光流叠图（裁剪掉 letterbox 黑边，只留有效行区，箭头看得清）
下：同一次运行的 |u| 分布 —— 看位移有多少被搜索窗硬顶住
"""
import os
import time
import numpy as np
import cv2
from pynq import Overlay, allocate

F_AP, F_IN, F_TH, F_OUT, F_R, F_C = 0x00, 0x10, 0x1c, 0x24, 0x30, 0x38
S_AP, S_PREV, S_CUR, S_COR, S_FLOW, S_ST = 0x00, 0x10, 0x1c, 0x28, 0x34, 0x40
S_N, S_ROWS, S_COLS, S_ITER = 0x4c, 0x54, 0x5c, 0x64
W, H = 640, 480
PMAX, Q, MAXIT, NC, SAT = 10, 6, 20, 100, 8.0
BIT = '/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/share/lo_sam/bitstreams/feature_extraction.bit'
IMGDIR = '/home/ubuntu/kitti_cam/image_02/data'
CROP0, CROP1 = 128, 352          # 有效行区 143..336 上下各留一点
PAIRS = [(55, 56), (95, 96), (127, 128)]
SEGN = [("55->56 turning", (117, 158, 29)), ("95->96 mild", (165, 95, 24)),
        ("127->128 sharp", (23, 117, 186))]
BINS = ['0-2', '2-4', '4-6', '6-7', '7-8', '8-9', '>9']


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
    return canvas, cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)


def txt(img, s, x, y, sc=0.38, col=(215, 215, 215)):
    cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, sc, col, 1, cv2.LINE_AA)


ol = Overlay(BIT, download=True)
fm, sm = ol.fast_accel_0.mmio, ol.slk_accel_0.mmio
inp = allocate(shape=(H, W), dtype=np.uint8, cacheable=1)
out = allocate(shape=(H, W), dtype=np.uint8, cacheable=1)
pb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cor_b = allocate((NC,), dtype=np.uint32, cacheable=1)
fl_b = allocate((NC,), dtype=np.uint32, cacheable=1)
st_b = allocate((NC,), dtype=np.uint32, cacheable=1)
wr64(fm, F_IN, inp.device_address); wr64(fm, F_OUT, out.device_address)
fm.write(F_TH, 20); fm.write(F_R, H); fm.write(F_C, W)
wr64(sm, S_PREV, pb.device_address); wr64(sm, S_CUR, cb.device_address)
wr64(sm, S_COR, cor_b.device_address); wr64(sm, S_FLOW, fl_b.device_address)
wr64(sm, S_ST, st_b.device_address)
sm.write(S_ROWS, H); sm.write(S_COLS, W); sm.write(S_ITER, MAXIT)

panels, hists = [], []
for (ka, kb) in PAIRS:
    prev_col, prev = grab(ka)
    _, cur = grab(kb)
    inp[:] = prev; inp.sync_to_device(); out.fill(0); out.sync_to_device()
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
    N = len(sel)
    pb[:] = pack(prev); pb.sync_to_device(); cb[:] = pack(cur); cb.sync_to_device()
    cor_b[:N] = (sel[:, 1].astype(np.uint32) << 16) | (sel[:, 0].astype(np.uint32) & 0xFFFF)
    cor_b.sync_to_device(); sm.write(S_N, N); sm.write(S_AP, 1)
    while not ((sm.read(S_AP) >> 1) & 1):
        time.sleep(0.0002)
    fl_b.sync_from_device(); st_b.sync_from_device()
    raw = np.array(fl_b).view(np.int16).reshape(NC, 2)
    u = raw[:N, 1].astype(np.float32) / (1 << Q)
    v = raw[:N, 0].astype(np.float32) / (1 << Q)
    st = np.array(st_b)[:N].astype(np.int32)

    canvas = prev_col.copy()
    ok = st == 1
    sat = ok & (np.abs(u) >= SAT)
    for i in np.nonzero(~ok)[0]:
        cv2.circle(canvas, (int(sel[i, 0]), int(sel[i, 1])), 2, (135, 135, 135), -1)
    for i in np.nonzero(ok & ~sat)[0]:
        p0 = (int(sel[i, 0]), int(sel[i, 1]))
        p1 = (int(round(sel[i, 0] + u[i])), int(round(sel[i, 1] + v[i])))
        cv2.arrowedLine(canvas, p0, p1, (140, 220, 60), 1, tipLength=0.34)
        cv2.circle(canvas, p0, 2, (60, 200, 40), -1)
    for i in np.nonzero(sat)[0]:
        p0 = (int(sel[i, 0]), int(sel[i, 1]))
        p1 = (int(round(sel[i, 0] + u[i])), int(round(sel[i, 1] + v[i])))
        cv2.arrowedLine(canvas, p0, p1, (60, 60, 235), 1, tipLength=0.34)
        cv2.circle(canvas, p0, 2, (40, 40, 230), -1)

    crop = canvas[CROP0:CROP1]
    bar = np.zeros((40, W, 3), np.uint8)
    nsat, nv = int(sat.sum()), int(ok.sum())
    mu = float(np.median(u[ok])) if ok.any() else float('nan')
    txt(bar, 'frames %d -> %d     corners %d  (FAST raw %d)     valid %d     saturated %d (%.0f%%)'
        % (ka, kb, N, n_all, nv, nsat, 100.0 * nsat / max(1, nv)), 10, 16, 0.42, (235, 235, 235))
    txt(bar, 'med_u %+.2f px     d_yaw %+.2f deg  (=-med_u/371.8)' % (mu, -mu / 371.8 * 57.2958),
        10, 33, 0.40, (160, 200, 160))
    panels.append(np.vstack([bar, crop]))

    uo = np.abs(u[ok])
    h, _ = np.histogram(uo, bins=[0, 2, 4, 6, 7, 8, 9, 1e9])
    hists.append(100.0 * h / max(1, len(uo)))
    print('frames %d->%d  n=%d  sat %.0f%%  hist %s' % (ka, kb, nv, 100.0 * nsat / max(1, nv), np.round(hists[-1], 1)))

LEG = 42
leg = np.zeros((LEG, W, 3), np.uint8)
txt(leg, 'green = flow inside the +/-9 px search window      RED = saturated at window edge (value unreliable)      grey = status 0',
    10, 16, 0.38, (225, 225, 225))
txt(leg, 'background = frame k (grayscale of the PL input, letterbox cropped)      arrow = corner at k  ->  tracked position at k+1',
    10, 34, 0.38, (185, 195, 205))

HP = 258
hp = np.zeros((HP, W, 3), np.uint8)
txt(hp, "|u| distribution (% of valid points)", 10, 20, 0.46, (235, 235, 235))
for si, (nm, col) in enumerate(SEGN):
    x = 296 + si * 112
    cv2.rectangle(hp, (x, 32), (x + 12, 44), col, -1)
    txt(hp, nm, x + 17, 41, 0.34, (215, 215, 215))
X0, X1, Y0, Y1, YMAX = 64, 624, 60, 196, 45.0
gw = (X1 - X0) / float(len(BINS))
band0, band1 = int(X0 + 4 * gw), int(X0 + 6 * gw)
cv2.rectangle(hp, (band0, Y0), (band1, Y1), (46, 41, 38), -1)
txt(hp, 'search-window edge', band0 + 8, Y0 + 14, 0.34, (150, 165, 200))
for v in (0, 10, 20, 30, 40):
    y = int(Y1 - v / YMAX * (Y1 - Y0))
    cv2.line(hp, (X0, y), (X1, y), (68, 68, 68), 1)
    txt(hp, '%d%%' % v, 26, y + 4, 0.38, (170, 170, 170))
cv2.line(hp, (X0, Y1), (X1, Y1), (140, 140, 140), 1)
bw, gap = 15, 3
for gi, lab in enumerate(BINS):
    gx = X0 + gi * gw
    for si, (nm, col) in enumerate(SEGN):
        hh = int(hists[si][gi] / YMAX * (Y1 - Y0))
        bx = int(gx + gw / 2 - (3 * bw + 2 * gap) / 2 + si * (bw + gap))
        if hh > 0:
            cv2.rectangle(hp, (bx, Y1 - hh), (bx + bw, Y1), col, -1)
    txt(hp, lab, int(gx + gw / 2 - 11), Y1 + 17, 0.38, (200, 200, 200))
txt(hp, '|u| (px)', int((X0 + X1) / 2) - 22, Y1 + 36, 0.40, (200, 200, 200))
txt(hp, 'no point exceeds 9 px  ==>  the LK search window (WH=5 / SEARCH=4) is the hard ceiling',
    10, 248, 0.40, (140, 180, 235))

img = np.vstack([leg] + panels + [hp])
cv2.imwrite('/home/ubuntu/pl_vision/viz/flow_and_hist.png', img)
print('WROTE', img.shape)
