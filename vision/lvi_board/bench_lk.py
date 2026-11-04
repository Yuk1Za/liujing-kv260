#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_lk.py —— 用「迭代上限扫描」把 LK 的固定开销与每次迭代成本分离。

背景：4 路 + eps 0.05 的内核在板上仍要 20.4 ms（60 点 × 3 层），
按综合报告的拍数推算「应该 ~8 ms」。两种可能：
  (a) 收敛慢 ⇒ 实际迭代次数仍有 13–14 次（eps 没起作用或有极限环）；
  (b) 每点每层的固定开销（prev patch 载入 + Sobel + A/G 矩阵）占大头。
本脚本不改代码、不重综合，只用 **max_iter 上限扫描** 区分二者：
  T(cap) ≈ 固定 + min(cap, 实际迭代数) × 单次迭代
  ⇒ cap=1 给的几乎就是「固定开销」；T(cap) 饱和处的 cap 就是实际迭代次数。

用法：
  sudo -n -E env HOME=/home/ubuntu PL_BIT=/home/ubuntu/vis3/design_1_wrapper_stlk4.bit PL_Q=4 \
       bash -c 'set +u; source /opt/ros/humble/setup.bash; set -u; python3 bench_lk.py'
"""
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, '/home/ubuntu/lvi_board')
from pl_match import PLMatcher, BIT  # noqa: E402
from vo_pnp_node import grab_img  # noqa: E402

H, W = 480, 640
MAXCNT, MIND = 60, 20
CAPS = [int(x) for x in os.environ.get('CAPS', '1,2,4,8,16,30').split(',')]
LEVS = [int(x) for x in os.environ.get('LEVS', '3').split(',')]
PAIRS = [(0, 1), (60, 61), (300, 301), (600, 601)]
DET = os.environ.get('DET', 'st')


def log(*a):
    print(*a)
    sys.stdout.flush()


def main():
    gs = {}
    for a, b in PAIRS:
        gs[a] = cv2.cvtColor(grab_img(a, W, H), cv2.COLOR_BGR2GRAY)
        gs[b] = cv2.cvtColor(grab_img(b, W, H), cv2.COLOR_BGR2GRAY)

    # cv2 参考光流（同参数），用于看不同 cap 下的精度损失
    ref = {}
    for a, b in PAIRS:
        p0 = cv2.goodFeaturesToTrack(gs[a], MAXCNT, 0.01, MIND)
        p1, st, _ = cv2.calcOpticalFlowPyrLK(gs[a], gs[b], p0, None, winSize=(21, 21), maxLevel=3)
        st = st.reshape(-1).astype(bool)
        ref[(a, b)] = (p0.reshape(-1, 2)[st], p1.reshape(-1, 2)[st])

    log('lev  cap   LK ms   跟踪点  流误差中位')
    base = {}
    for lv in LEVS:
      for cap in CAPS:
        m = PLMatcher(bit=BIT, max_cnt=MAXCNT, min_dist=MIND, levels=lv, max_iter=cap)
        lks, ns, errs = [], [], []
        for a, b in PAIRS:
            r = m.match(gs[a], gs[b], det=DET)
            lks.append(r['t']['lk'])
            ns.append(len(r['uv_prev']))
            q0, q1 = ref[(a, b)]
            up, uc = r['uv_prev'], r['uv_cur']
            d = np.linalg.norm(up[:, None, :] - q0[None, :, :], axis=2)
            j = d.argmin(1)
            ok = d[np.arange(len(up)), j] <= 1.5
            if ok.any():
                errs.append(np.median(np.linalg.norm(uc[ok] - q1[j[ok]], axis=1)))
        base[(lv, cap)] = np.median(lks)
        log(f'{lv:3d} {cap:4d} {np.median(lks):8.2f} {np.median(ns):7.0f} '
            f'{np.median(errs) if errs else float("nan"):11.3f}')

    for lv in LEVS:
        if (lv, 1) in base:
            t1 = base[(lv, 1)]
            tm = base[(lv, max(CAPS))]
            log(f'\nL{lv}: 固定开销(cap=1) {t1:.2f} ms，cap={max(CAPS)} 时 {tm:.2f} ms '
                f'⇒ 迭代部分 {tm - t1:.2f} ms')


if __name__ == '__main__':
    main()
