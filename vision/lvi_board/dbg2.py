#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断：把「粗层初值错」和「最细层收敛不了」分开。
对每对帧，用同一批 prev 点：
  A) levels=1, init=0                → 单层能覆盖多大流
  B) levels=1, init=cv2 真值         → 最细层在给定初值下的精度上限
  C) levels=3 / 4, init=0            → 整条链
按 cv2 真实流大小分桶统计误差，看是哪一段塌掉。
"""
import sys

import cv2
import numpy as np

sys.path.insert(0, '/home/ubuntu/lvi_board')
IMG = '/home/ubuntu/kitti_cam/image_02/data'


def gray(i):
    im = cv2.imread(f'{IMG}/{i:010d}.png')
    nh = int(round(im.shape[0] * 640 / im.shape[1]))
    r = cv2.resize(im, (640, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((480, 640, 3), np.uint8)
    top = (480 - nh) // 2
    out[top:top + nh] = r
    return cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)


from pl_match import PLMatcher

PAIRS = [11, 23, 31, 7, 3, 63, 99, 131]
ms = {k: PLMatcher(levels=k, max_iter=5) for k in (1, 3, 4)}

BUCK = [(0, 2), (2, 5), (5, 10), (10, 20), (20, 1e9)]
acc = {}

for k in PAIRS:
    g1, g2 = gray(k), gray(k + 1)
    for lv in (1, 3, 4):
        ms[lv].match(g1, g2)          # 先把图上传进各自的 buffer
    r = ms[4].match(g1, g2)
    pts = r['uv_prev']
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, pts.reshape(-1, 1, 2).astype(np.float32), None,
                                        winSize=(21, 21), maxLevel=3)
    fcv = p1.reshape(-1, 2) - pts
    okc = st.reshape(-1).astype(bool)
    mag = np.linalg.norm(fcv, axis=1)

    # A) 单层 init=0
    fA, okA = ms[1]._run_lk(0, pts.astype(np.uint32), np.zeros((len(pts), 2)), len(pts))
    eA = np.linalg.norm(fA - fcv, axis=1)
    # B) 单层 init=真值
    fB, okB = ms[1]._run_lk(0, pts.astype(np.uint32), fcv.copy(), len(pts))
    eB = np.linalg.norm(fB - fcv, axis=1)
    # C) 链
    for lv in (3, 4):
        rr = ms[lv].match(g1, g2)
        # 与 pts 同序：match 内部 ok 过滤后可能少点，这里用 order 对齐
        fpl = rr['uv_cur'] - rr['uv_prev']
        # 按 prev 坐标精确匹配回 pts
        d = np.linalg.norm(rr['uv_prev'][:, None, :] - pts[None, :, :], axis=2)
        j = d.argmin(1)
        good = d[np.arange(len(j)), j] < 1e-6
        e = np.full(len(pts), np.nan)
        e[j[good]] = np.linalg.norm(fpl[good] - fcv[j[good]], axis=1)
        acc.setdefault(f'C L{lv}', []).append(e)
    acc.setdefault('A 单层 init0', []).append(eA)
    acc.setdefault('B 单层 init真值', []).append(eB)
    acc.setdefault('mag', []).append(mag)
    acc.setdefault('okA', []).append(okA)

mag = np.concatenate(acc['mag'])
print('按 cv2 真实流大小分桶（误差 = |PL 流 − cv2 流|，px，中位）')
print(f'{"":18s}' + ''.join(f'[{lo},{hi if hi<1e8 else "∞"})'.rjust(13) for lo, hi in BUCK))
for name in ('A 单层 init0', 'B 单层 init真值', 'C L3', 'C L4'):
    e = np.concatenate(acc[name])
    line = f'{name:18s}'
    for lo, hi in BUCK:
        m = (mag >= lo) & (mag < hi) & np.isfinite(e)
        line += (f'{np.median(e[m]):8.2f}({m.sum():3d})' if m.sum() else '          -  ')
    print(line)
print()
print('各桶点数:', {f'[{lo},{hi if hi<1e8 else "∞"})': int(((mag >= lo) & (mag < hi)).sum())
                   for lo, hi in BUCK})
print('单层 init0 跟踪率: %.0f%%' % (100 * np.concatenate(acc['okA']).mean()))
