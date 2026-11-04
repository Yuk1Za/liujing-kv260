#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补测：① cv2 分支里 GFTT 与 LK 各占多少  ② 读图的加速选项（IMREAD_REDUCED_*）
"""
import time

import cv2
import numpy as np

IMG = '/home/ubuntu/kitti_cam/image_02/data'


def gray(k):
    im = cv2.imread(f'{IMG}/{k:010d}.png')
    nh = int(round(im.shape[0] * 640 / im.shape[1]))
    r = cv2.resize(im, (640, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((480, 640, 3), np.uint8)
    out[(480 - nh) // 2:(480 - nh) // 2 + nh] = r
    return cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)


g = {k: gray(k) for k in (3, 4, 19, 20, 63, 64, 131, 132)}
tg, tl, ng = [], [], []
for k in (3, 19, 63, 131):
    g1, g2 = g[k], g[k + 1]
    for _ in range(3):
        t0 = time.perf_counter()
        p0 = cv2.goodFeaturesToTrack(g1, 60, 0.05, 20.0)
        tg.append((time.perf_counter() - t0) * 1e3)
        ng.append(len(p0))
        t0 = time.perf_counter()
        cv2.calcOpticalFlowPyrLK(g1, g2, p0, None, winSize=(21, 21), maxLevel=3)
        tl.append((time.perf_counter() - t0) * 1e3)
print(f'① cv2 分支: GFTT {np.median(tg):.2f} ms (检出 {int(np.median(ng))})  '
      f'+ LK {np.median(tl):.2f} ms  = {np.median(tg)+np.median(tl):.2f} ms')

print('② 读图选项（单帧，ms）:')
opts = [('imread 全尺寸 + resize', lambda k: cv2.resize(cv2.imread(f'{IMG}/{k:010d}.png'), (640, 193),
                                                        interpolation=cv2.INTER_AREA)),
        ('IMREAD_REDUCED_COLOR_2', lambda k: cv2.imread(f'{IMG}/{k:010d}.png',
                                                        cv2.IMREAD_REDUCED_COLOR_2)),
        ('IMREAD_REDUCED_COLOR_4', lambda k: cv2.imread(f'{IMG}/{k:010d}.png',
                                                        cv2.IMREAD_REDUCED_COLOR_4))]
for name, fn in opts:
    t = []
    for k in range(10):
        t0 = time.perf_counter()
        x = fn(k)
        t.append((time.perf_counter() - t0) * 1e3)
    print(f'   {name:26s} {np.median(t):6.2f} ms  输出 {x.shape}')
