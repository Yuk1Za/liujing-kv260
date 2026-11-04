#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线 A/B：PL 前端 vs cv2 前端（同一批 50 对相邻帧，与 ROS 里 every=4 的对完全一致）。
只量「匹配这一段」的耗时与流一致性；端到端精度由 ROS A/B 量。
"""
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, '/home/ubuntu/lvi_board')
H = '/home/ubuntu'
IMG = f'{H}/kitti_cam/image_02/data'
MAX_CNT = 60
PAIRS = list(range(3, 200, 4))


def gray(i):
    im = cv2.imread(f'{IMG}/{i:010d}.png')
    nh = int(round(im.shape[0] * 640 / im.shape[1]))
    r = cv2.resize(im, (640, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((480, 640, 3), np.uint8)
    top = (480 - nh) // 2
    out[top:top + nh] = r
    return cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)


G = {i: gray(i) for i in sorted(set(PAIRS) | {i + 1 for i in PAIRS})}
print(f'缓存 {len(G)} 帧灰度图')


def cv2_match(g1, g2):
    t0 = time.perf_counter()
    p0 = cv2.goodFeaturesToTrack(g1, MAX_CNT, 0.05, 20.0)
    if p0 is None:
        return np.zeros((0, 2)), np.zeros((0, 2)), (time.perf_counter() - t0) * 1e3
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0, None, winSize=(21, 21), maxLevel=3)
    st = st.reshape(-1).astype(bool)
    return (p0.reshape(-1, 2)[st], p1.reshape(-1, 2)[st], (time.perf_counter() - t0) * 1e3)


def cv2_lk_from(g1, g2, p0):
    """用给定点集跑 cv2 LK（用来和 PL 在**同一批点**上比流）。"""
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0.reshape(-1, 1, 2).astype(np.float32),
                                        None, winSize=(21, 21), maxLevel=3)
    return p1.reshape(-1, 2), st.reshape(-1).astype(bool)


from pl_match import PLMatcher

CFG = [(3, 5), (4, 5), (4, 10)]
M = {}
for c in CFG:
    M[c] = PLMatcher(max_cnt=MAX_CNT, min_dist=20.0, levels=c[0], max_iter=c[1])
    print(f'PL 就绪 levels={c[0]} max_iter={c[1]}')
for m in M.values():
    m.match(G[3], G[4])          # warmup

print()
print('=== 匹配段耗时（50 对）===')
rows = {}
t_cv = []
for k in PAIRS:
    _, _, dt = cv2_match(G[k], G[k + 1])
    t_cv.append(dt)
rows['cv2 (gftt+LK pyr)'] = np.array(t_cv)
for c, m in M.items():
    d, n = [], []
    for k in PAIRS:
        r = m.match(G[k], G[k + 1])
        d.append(r['t']['total'])
        n.append(len(r['uv_prev']))
    rows[f'PL L{c[0]} it{c[1]}'] = np.array(d)
    print(f'  PL L{c[0]} it{c[1]}: 匹配点数中位 {int(np.median(n))}/{MAX_CNT}')

for name, d in rows.items():
    print(f'  {name:20s} 中位 {np.median(d):6.2f} ms  p90 {np.percentile(d,90):6.2f}  '
          f'max {d.max():6.2f}')

print()
print('=== PL 分段（levels=4, max_iter=5）===')
det = []
for k in PAIRS:
    r = M[(4, 5)].match(G[k], G[k + 1])
    det.append(r['t'])
for k in ('pyramid', 'upload', 'fast', 'select', 'lk', 'total'):
    a = np.array([d[k] for d in det])
    print(f'  {k:9s} 中位 {np.median(a):6.2f} ms  均值 {a.mean():6.2f}')

print()
print('=== 同一批 prev 点上：PL 流 vs cv2 流（隔离匹配算法差异）===')
for c in CFG:
    errs, rates = [], []
    for k in PAIRS:
        r = M[c].match(G[k], G[k + 1])
        upp, ucp = r['uv_prev'], r['uv_cur']
        if len(upp) < 5:
            rates.append(0.0)
            continue
        p1c, okc = cv2_lk_from(G[k], G[k + 1], upp)
        good = okc
        rates.append(good.mean())
        if good.sum() < 5:
            continue
        fp = ucp - upp
        fc = p1c - upp
        e = np.linalg.norm(fp[good] - fc[good], axis=1)
        errs.append(np.median(e))
    errs = np.array(errs)
    print(f'  L{c[0]} it{c[1]}: cv2 在这些点上成功率 {100*np.median(rates):.0f}% | '
          f'逐对流差中位 {np.median(errs):.3f} px  p90 {np.percentile(errs,90):.3f}  max {errs.max():.2f}')
