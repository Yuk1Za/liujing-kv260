#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_sel.py —— 板上拆解 Shi-Tomasi 检点的「IP + 后处理」各段真实耗时。

目的只有一个：27 ms 的 select 到底花在哪、有没有便宜的改法。
按项目惯例：**先量化再改代码**，别凭直觉优化。

测的项目（都在板端真机）：
  A. ST IP 本体
  B. rbuf→numpy 拷贝 + 边界抹零
  C. cv2.findNonZero
  D. argsort（按响应降序）
  E. 网格屏蔽贪心（select_min_dist_fast），并扫「候选截断 K」
  F. 参考实现（n_cand × max_corners 那版）用于对照
  G. 与 cv2.goodFeaturesToTrack 的点集重合率（验证截断有没有把结果带偏）

用法：
  sudo -n -E env HOME=/home/ubuntu PL_BIT=/home/ubuntu/vis3/design_1_wrapper_stlk4.bit \
       bash -c 'set +u; source /opt/ros/humble/setup.bash; set -u; python3 bench_sel.py'
"""
import os
import sys
import time

import numpy as np
import cv2

sys.path.insert(0, '/home/ubuntu/lvi_board')
from pl_match import (PLMatcher, BIT, select_min_dist, select_min_dist_fast,  # noqa: E402
                      select_min_dist_k)
from vo_pnp_node import grab_img  # noqa: E402

H, W = 480, 640
MAXCNT, MIND, PMAX = 60, 20, 10
FRAMES = [int(x) for x in os.environ.get('FRAMES', '0,60,300,600').split(',')]


def log(*a):
    print(*a)
    sys.stdout.flush()


def rep(a, b, tol=2.0):
    if len(a) == 0 or len(b) == 0:
        return 0.0
    d = np.linalg.norm(a[:, None, :].astype(np.float64) - b[None, :, :].astype(np.float64), axis=2)
    return float((d.min(1) <= tol).mean())


def main():
    m = PLMatcher(bit=BIT, max_cnt=MAXCNT, min_dist=MIND, levels=3)
    log(f'PL IP: {m.ips}')
    m.detect(cv2.cvtColor(grab_img(FRAMES[0], W, H), cv2.COLOR_BGR2GRAY), det='st')   # 预热

    rows = []
    for f in FRAMES:
        g = cv2.cvtColor(grab_img(f, W, H), cv2.COLOR_BGR2GRAY)
        m.detect(g, det='st')
        t_ip = m.t['st']

        t0 = time.perf_counter()
        resp = np.array(m.rbuf).view(np.uint8).reshape(H, W)
        resp[:PMAX, :] = 0
        resp[H - PMAX:, :] = 0
        resp[:, :PMAX] = 0
        resp[:, W - PMAX:] = 0
        t_copy = (time.perf_counter() - t0) * 1e3

        t0 = time.perf_counter()
        pos = cv2.findNonZero(resp).reshape(-1, 2)
        t_nz = (time.perf_counter() - t0) * 1e3

        # ★ cv2 参考必须**逐帧**算（上一版复用帧 0 的点集，导致后面几帧"重合 0%"的假象）
        t0 = time.perf_counter()
        gcv = cv2.goodFeaturesToTrack(g, MAXCNT, 0.01, MIND)
        gcv = np.zeros((0, 2)) if gcv is None else gcv.reshape(-1, 2)
        t_cv = (time.perf_counter() - t0) * 1e3

        t0 = time.perf_counter()
        Pr = select_min_dist(resp, MAXCNT, MIND, 0)
        t_ref = (time.perf_counter() - t0) * 1e3

        log(f'\n--- 帧 {f}  候选 {len(pos)} ---')
        log(f'  ST IP {t_ip:5.1f} | 拷贝+抹边 {t_copy:4.1f} | findNonZero {t_nz:4.1f} | '
            f'cv2 GFTT {t_cv:5.1f} | 选点(参考全量) {t_ref:5.1f}')
        log(f'  参考全量：{len(Pr[0]):3d} 点  重合 cv2 {rep(Pr[0], gcv)*100:5.1f}%')
        for K in (300, 600, 900, 1200):
            t0 = time.perf_counter()
            pk, nc, nd = select_min_dist_k(resp, MAXCNT, MIND, K)
            tk = (time.perf_counter() - t0) * 1e3
            log(f'  截断 K={K:4d}: {tk:6.2f} ms  {len(pk):3d} 点  重合 cv2 {rep(pk, gcv)*100:5.1f}%')
        rows.append((t_ip, t_copy, t_nz, t_ref, t_cv))

    A = np.array(rows)
    log('\n=== 中位 ===')
    log(f'  ST IP {np.median(A[:,0]):.1f} | 拷贝 {np.median(A[:,1]):.1f} | nonzero {np.median(A[:,2]):.1f} | '
        f'选点(参考全量) {np.median(A[:,3]):.1f} | cv2 GFTT {np.median(A[:,4]):.1f} ms')


if __name__ == '__main__':
    main()
