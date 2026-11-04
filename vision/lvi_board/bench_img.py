#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_img.py —— 对比「原图在线解码」与「预缩放灰度直读」的真实成本（板端）。"""
import sys, time
import numpy as np, cv2
sys.path.insert(0, '/home/ubuntu/lvi_board')
import vo_pnp_node as N

FR = [0, 40, 80, 120, 160, 200, 240, 280]
def bench(tag, fns):
    ts = []
    for f in FR:
        t0 = time.perf_counter()
        for _ in range(2):           # 每唤醒两张图
            fns(f)
        ts.append((time.perf_counter()-t0)*1e3)
    print(f'  {tag}: 中位 {np.median(ts):6.1f} ms/唤醒  (min {min(ts):.1f} max {max(ts):.1f})')
    return float(np.median(ts))

def old(f):
    im = N.grab_img(f, 640, 480)
    return None if im is None else cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

N.IMG_PRE = '/home/ubuntu/kitti_pre'
def new(f):
    return N.grab_gray(f, 640, 480)

print('=== 读图成本（每唤醒 2 张）===')
a = bench('原图 1226×370 彩色 → resize → 灰度', old)
b = bench('预缩放 640×480 灰度直读        ', new)
print(f'\n节省 {a-b:.1f} ms/唤醒（{a:.1f} → {b:.1f}）')
# 一致性
d = []
for f in FR:
    x, y = old(f), new(f)
    d.append(bool(np.array_equal(x, y)))
print('逐位一致:', f'{sum(d)}/{len(d)}')
