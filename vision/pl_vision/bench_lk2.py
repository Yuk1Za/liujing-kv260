#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LK 基准 2：按「位移大小」分档，看 PL 的每点耗时怎么变
小位移 = Wheeltec 慢速场景；真实 KITTI 相邻帧 = 城市驾驶（位移 5~8 px）
"""
import os, time
import numpy as np, cv2
from pynq import Overlay, allocate

S_AP, S_PREV, S_CUR, S_COR, S_FLOW, S_ST = 0x00, 0x10, 0x1c, 0x28, 0x34, 0x40
S_N, S_ROWS, S_COLS, S_ITER = 0x4c, 0x54, 0x5c, 0x64
W, H, PMAX, Q, MAXIT, NC, REP = 640, 480, 10, 6, 20, 300, 15
BIT = '/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/share/lo_sam/bitstreams/feature_extraction.bit'
IMGDIR = '/home/ubuntu/kitti_cam/image_02/data'


def wr64(mm, a, v):
    mm.write(a, v & 0xFFFFFFFF); mm.write(a + 4, (v >> 32) & 0xFFFFFFFF)


def pack(img):
    r, c = img.shape
    return img.reshape(r, c // 4, 4).copy().view(np.uint32).reshape(r, c // 4)


def grab(i):
    img = cv2.imread(os.path.join(IMGDIR, '%010d.png' % i), cv2.IMREAD_COLOR)
    h0, w0 = img.shape[:2]
    h2 = min(H, max(1, int(round(h0 * W / float(w0)))))
    small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
    cv = np.zeros((H, W, 3), np.uint8); top = (H - h2) // 2
    cv[top:top + h2] = small
    return cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)


def med(fn, rep=REP):
    fn(); ts = []
    for _ in range(rep):
        t0 = time.perf_counter(); fn(); ts.append((time.perf_counter() - t0) * 1e3)
    return float(np.median(ts))


ol = Overlay(BIT, download=True)
sm = ol.slk_accel_0.mmio
pb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
cor_b = allocate((NC,), dtype=np.uint32, cacheable=1)
fl_b = allocate((NC,), dtype=np.uint32, cacheable=1)
st_b = allocate((NC,), dtype=np.uint32, cacheable=1)
wr64(sm, S_PREV, pb.device_address); wr64(sm, S_CUR, cb.device_address)
wr64(sm, S_COR, cor_b.device_address); wr64(sm, S_FLOW, fl_b.device_address)
wr64(sm, S_ST, st_b.device_address)
sm.write(S_ROWS, H); sm.write(S_COLS, W); sm.write(S_ITER, MAXIT); sm.write(S_N, NC)

rng = np.random.default_rng(0)
xs = rng.uniform(PMAX + 2, W - PMAX - 3, NC)
ys = rng.uniform(143 + PMAX + 2, 336 - PMAX - 2, NC)
pts = np.stack([xs, ys], 1).astype(np.float32)
pts_int = pts.astype(np.int32)
prev = grab(95)

CASES = [
    ('synthetic shift (1, 0.5) px', cv2.warpAffine(prev, np.float32([[1, 0, 1], [0, 1, 0.5]]), (W, H), flags=cv2.INTER_LINEAR)),
    ('synthetic shift (3, 1.5) px', cv2.warpAffine(prev, np.float32([[1, 0, 3], [0, 1, 1.5]]), (W, H), flags=cv2.INTER_LINEAR)),
    ('synthetic shift (6, 3) px',   cv2.warpAffine(prev, np.float32([[1, 0, 6], [0, 1, 3]]),   (W, H), flags=cv2.INTER_LINEAR)),
    ('KITTI 95 -> 96 (real)',       grab(96)),
    ('KITTI 127 -> 128 (real)',     grab(128)),
]
print('%-30s %10s %10s %10s %10s %10s' % ('case', 'PL ms', 'PL us/pt', 'CPU1 ms', 'CPU4 ms', 'median|u|'))
for tag, cur in CASES:
    pb[:] = pack(prev); pb.sync_to_device(); cb[:] = pack(cur); cb.sync_to_device()
    cor_b[:NC] = (pts_int[:, 1].astype(np.uint32) << 16) | (pts_int[:, 0].astype(np.uint32) & 0xFFFF)
    cor_b.sync_to_device()

    def ip():
        sm.write(S_AP, 1)
        while not ((sm.read(S_AP) >> 1) & 1):
            time.sleep(0.0002)
    t_pl = med(ip)
    fl_b.sync_from_device(); st_b.sync_from_device()
    u = np.array(fl_b).view(np.int16).reshape(NC, 2)[:, 1].astype(np.float32) / (1 << Q)
    st = np.array(st_b)
    mu = float(np.median(np.abs(u[st == 1]))) if (st == 1).any() else float('nan')

    def c1():
        cv2.setNumThreads(1)
        cv2.calcOpticalFlowPyrLK(prev, cur, pts, None, winSize=(21, 21), maxLevel=2,
                                 criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 10, 0.05))
    t1 = med(c1)

    def c4():
        cv2.setNumThreads(4)
        cv2.calcOpticalFlowPyrLK(prev, cur, pts, None, winSize=(21, 21), maxLevel=2,
                                 criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 10, 0.05))
    t4 = med(c4)
    cv2.setNumThreads(4)
    print('%-30s %10.2f %10.1f %10.2f %10.2f %10.2f' % (tag, t_pl, t_pl * 1e3 / NC, t1, t4, mu))
