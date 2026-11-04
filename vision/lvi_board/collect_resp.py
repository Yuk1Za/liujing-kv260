#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""collect_resp.py —— 导出 PL Shi-Tomasi 的真实响应图 + Python 参考选点结果。

给 HLS 的 NMS 内核做**逐点等价性**对拍用（不是近似、不是形状相似）：
  · resp_XX.bin  : 480×640 uint8 原始响应图（未经任何屏蔽/阈值）
  · ref_XX.txt   : 参考实现的输出点（按响应降序贪心 + MIN_DIST 欧氏抑制 + 前 60）
两边都必须从**同一张原始响应图**出发，各自做 border 屏蔽 / 阈值 / 贪心，才能比点集。

用法（板端，需 root）：
  sudo -n -E env HOME=/home/ubuntu PL_BIT=... python3 collect_resp.py [起始帧] [张数]
输出目录：/home/ubuntu/lvi_board/nms_ab/
"""
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, '/home/ubuntu/lvi_board')
from pl_match import PLMatcher, BIT, select_min_dist  # noqa: E402
from vo_pnp_node import grab_img  # noqa: E402

H, W, PMAX, MAXCNT, MIND = 480, 640, 10, 60, 20
OUT = '/home/ubuntu/lvi_board/nms_ab'


def main():
    f0 = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    os.makedirs(OUT, exist_ok=True)
    m = PLMatcher(bit=BIT, max_cnt=MAXCNT, min_dist=MIND, levels=2, max_iter=8)

    for k in range(n):
        f = f0 + k * 60
        g = cv2.cvtColor(grab_img(f, W, H), cv2.COLOR_BGR2GRAY)
        m.detect(g, det='st')
        resp = np.array(m.rbuf).view(np.uint8).reshape(H, W).copy()   # 原始响应图
        resp.tofile(f'{OUT}/resp_{f:04d}.bin')

        # 参考路径：与 _detect_hw 的 st 分支完全一致
        r = resp.copy()
        r[:PMAX, :] = 0
        r[H - PMAX:, :] = 0
        r[:, :PMAX] = 0
        r[:, W - PMAX:] = 0
        thr = max(1, int(round(0.01 * int(r.max()))))
        r[r < thr] = 0
        pts, n_cand, _ = select_min_dist(r, MAXCNT, MIND, 0)
        np.savetxt(f'{OUT}/ref_{f:04d}.txt', pts, fmt='%d')
        # 阈值与候选数写进 meta，C 测试台要按同一组标量跑
        with open(f'{OUT}/meta_{f:04d}.txt', 'w') as fp:
            fp.write(f'{thr} {n_cand} {len(pts)}\n')
        print(f'帧 {f}: 阈值 {thr} 候选 {n_cand} → {len(pts)} 点')
    print('输出目录', OUT)


if __name__ == '__main__':
    main()
