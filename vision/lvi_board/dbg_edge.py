#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量化：PL FAST 的角点里有多少落在 letterbox 黑边的硬边界上（y=143 / y=336）。"""
import sys

import cv2
import numpy as np

sys.path.insert(0, '/home/ubuntu/lvi_board')
from pl_match import PLMatcher

IMG = '/home/ubuntu/kitti_cam/image_02/data'


def gray(i):
    im = cv2.imread(f'{IMG}/{i:010d}.png')
    nh = int(round(im.shape[0] * 640 / im.shape[1]))
    r = cv2.resize(im, (640, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((480, 640, 3), np.uint8)
    top = (480 - nh) // 2
    out[top:top + nh] = r
    return cv2.cvtColor(out, cv2.COLOR_BGR2GRAY), top, nh


m = PLMatcher(levels=3, max_iter=5)
for k in (3, 19, 63, 131):
    g1, top, nh = gray(k)
    m.match(g1, g1)
    pts = m._fast_points()
    sel = m._select(pts)
    y = pts[:, 1]
    edge = ((np.abs(y - top) <= 2) | (np.abs(y - (top + nh - 1)) <= 2)).sum()
    print(f'帧{k}: 内容带 rows [{top},{top+nh})  FAST 检出 {len(pts)}，其中贴黑边 {edge} '
          f'({100*edge/max(len(pts),1):.0f}%)')
    ys = sel[:, 1]
    print(f'   选中的 60 个点: y 落黑边附近 {(np.abs(ys-top)<=2).sum() + (np.abs(ys-(top+nh-1))<=2).sum()} 个  '
          f'| y 范围 {ys.min():.0f}..{ys.max():.0f}  唯一 y 数 {len(np.unique(ys))}')
    print(f'   x 分布: min {sel[:,0].min():.0f} max {sel[:,0].max():.0f}  '
          f'| cv2 GFTT 选点 y 唯一数 {len(np.unique(cv2.goodFeaturesToTrack(g1, 60, 0.05, 20.0).reshape(-1,2)[:,1].round()))}')
