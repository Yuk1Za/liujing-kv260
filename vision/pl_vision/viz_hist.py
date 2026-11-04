#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""|u| 分布直方图：看位移有多少打在搜索窗边界上（饱和）"""
import numpy as np, cv2, os
exec(open('/home/ubuntu/pl_vision/viz_overlay.py').read().split('def main()')[0])
from pynq import Overlay, allocate

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

for ka, kb in [(55, 56), (95, 96), (127, 128)]:
    _, prev, _ = grab(ka); _, cur, _ = grab(kb)
    inp[:] = prev; inp.sync_to_device(); out.fill(0); out.sync_to_device()
    fm.write(F_AP, 1)
    while not ((fm.read(F_AP) >> 1) & 1): pass
    out.sync_from_device()
    ys, xs = np.nonzero(np.array(out))
    m = (xs >= PMAX) & (ys >= PMAX) & (xs < W - PMAX) & (ys < H - PMAX)
    sel = np.stack([xs[m], ys[m]], 1).astype(np.int32)
    if len(sel) > NC: sel = sel[np.linspace(0, len(sel) - 1, NC).astype(np.int64)]
    N = len(sel)
    pb[:] = pack(prev); pb.sync_to_device(); cb[:] = pack(cur); cb.sync_to_device()
    cor_b[:N] = (sel[:, 1].astype(np.uint32) << 16) | (sel[:, 0].astype(np.uint32) & 0xFFFF)
    cor_b.sync_to_device(); sm.write(S_N, N); sm.write(S_AP, 1)
    while not ((sm.read(S_AP) >> 1) & 1): pass
    fl_b.sync_from_device(); st_b.sync_from_device()
    raw = np.array(fl_b).view(np.int16).reshape(NC, 2)
    u = raw[:N, 1].astype(np.float32) / (1 << Q)
    st = np.array(st_b)[:N].astype(np.int32)
    uo = np.abs(u[st == 1])
    hist, edges = np.histogram(uo, bins=[0, 2, 4, 6, 7, 8, 9, 100])
    print("%d->%d  n=%d  bins(0-2,2-4,4-6,6-7,7-8,8-9,>9) = %s  >=8px: %.0f%%"
          % (ka, kb, len(uo), hist.tolist(), 100.0 * (uo >= 8).sum() / len(uo)))
