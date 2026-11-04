#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_sel2.py —— 校验 `select_min_dist_circle` 与参考实现**逐点等价**，并比速度。

关键判据：k=0（不截断）时，circle 版必须与参考版给出**完全相同的点集**，
否则"半径 min_dist−1 的圆盘 == cv2 的 d<min_dist 抑制集"这个论证就不成立。
只要等价，就可以放心用 circle 版（快 3–4×）。

用法：
  sudo -n -E env HOME=/home/ubuntu PL_BIT=/home/ubuntu/vis3/design_1_wrapper_stlk4.bit \
       bash -c 'set +u; source /opt/ros/humble/setup.bash; set -u; python3 bench_sel2.py'
"""
import sys
import time

import numpy as np
import cv2

sys.path.insert(0, '/home/ubuntu/lvi_board')
from pl_match import (PLMatcher, BIT, select_min_dist,  # noqa: E402
                      select_min_dist_k, select_min_dist_circle)
from vo_pnp_node import grab_img  # noqa: E402

H, W, PMAX, MAXCNT, MIND = 480, 640, 10, 60, 20
FRAMES = [0, 60, 300, 600, 900]


def log(*a):
    print(*a)
    sys.stdout.flush()


def same(a, b):
    if len(a) != len(b):
        return False
    return bool(np.array_equal(np.sort(a.view([('x', a.dtype), ('y', a.dtype)]), axis=0),
                               np.sort(b.view([('x', b.dtype), ('y', b.dtype)]), axis=0)))


def main():
    m = PLMatcher(bit=BIT, max_cnt=MAXCNT, min_dist=MIND, levels=2, max_iter=8)
    m.detect(cv2.cvtColor(grab_img(FRAMES[0], W, H), cv2.COLOR_BGR2GRAY), det='st')
    log(f'PL IP: {m.ips}')

    tr, tc, tc6, eq, eq6 = [], [], [], [], []
    for f in FRAMES:
        g = cv2.cvtColor(grab_img(f, W, H), cv2.COLOR_BGR2GRAY)
        m.detect(g, det='st')
        resp = np.array(m.rbuf).view(np.uint8).reshape(H, W)
        resp[:PMAX, :] = 0
        resp[H - PMAX:, :] = 0
        resp[:, :PMAX] = 0
        resp[:, W - PMAX:] = 0

        t0 = time.perf_counter()
        pr, nc, _ = select_min_dist(resp, MAXCNT, MIND, 0)
        t_ref = (time.perf_counter() - t0) * 1e3

        t0 = time.perf_counter()
        pc, _, _ = select_min_dist_circle(resp, MAXCNT, MIND, 0, H, W)
        t_cir = (time.perf_counter() - t0) * 1e3

        t0 = time.perf_counter()
        p6, _, _ = select_min_dist_circle(resp, MAXCNT, MIND, 600, H, W)
        t_cir6 = (time.perf_counter() - t0) * 1e3

        t0 = time.perf_counter()
        pk, _, _ = select_min_dist_k(resp, MAXCNT, MIND, 600)
        t_k = (time.perf_counter() - t0) * 1e3

        ec = same(pr, pc)
        e6 = same(pr, p6)
        tr.append(t_ref)
        tc.append(t_cir)
        tc6.append(t_cir6)
        eq.append(ec)
        eq6.append(e6)
        log(f'  帧 {f:3d} 候选 {nc:4d} | 参考 {t_ref:5.2f}ms {len(pr):2d}点 | '
            f'circle(k=0) {t_cir:5.2f}ms {len(pc):2d}点 {"✓逐点相同" if ec else "✗不同"} | '
            f'circle(k=600) {t_cir6:5.2f}ms {len(p6):2d}点 {"✓" if e6 else "✗"} | '
            f'k600 {t_k:5.2f}ms {len(pk):2d}点')

    log(f'\n中位：参考 {np.median(tr):.2f} | circle(k=0) {np.median(tc):.2f} | '
        f'circle(k=600) {np.median(tc6):.2f} ms')
    log(f'逐点等价：circle(k=0) {sum(eq)}/{len(eq)}   circle(k=600) {sum(eq6)}/{len(eq6)}')


if __name__ == '__main__':
    main()
