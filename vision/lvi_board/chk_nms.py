#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chk_nms.py —— 校验硬件 NMS（nms_accel_0）与 Python 参考选点**逐点逐序一致**，并给时序。

为什么必须查这个：NMS 内核的等价性已经在 g++/C-sim 上按「同一张响应图」验过 6/6，
但那是主机数据；上板后还有三件事可能出错 ——
  ① 寄存器偏移抄错（`resp/out_xy/out_n` 是 8 字节对，标量在 0x34 起）；
  ② 响应图基址没指到 ST 写的那块 DMA buffer；
  ③ 溢出判定 / 点数编码（out_n 的 bit31）。
所以这里在**同一帧、同一块响应图**上，分别用硬件与 Python 各选一次，逐点比。

用法（板端，需 root）：
  sudo -n -E env HOME=/home/ubuntu PL_BIT=/home/ubuntu/vis3/design_1_wrapper_stn.bit \
       bash -c 'set +u; source /opt/ros/humble/setup.bash; set -u; python3 chk_nms.py'
"""
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, '/home/ubuntu/lvi_board')
from pl_match import PLMatcher, BIT  # noqa: E402
from vo_pnp_node import grab_img  # noqa: E402

H, W, MAXCNT, MIND = 480, 640, 60, 20
FRAMES = [int(x) for x in os.environ.get('FRAMES', '0,60,120,180,240,300').split(',')]


def log(*a):
    print(*a)
    sys.stdout.flush()


def rep(a, b, tol=2.0):
    if len(a) == 0 or len(b) == 0:
        return 0.0
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    return float((d.min(1) <= tol).mean())


def main():
    m = PLMatcher(bit=BIT, max_cnt=MAXCNT, min_dist=MIND, levels=2, max_iter=8, nms='auto')
    log(f'PL IP: {m.ips}')
    log(f'NMS IP: {"有" if m.nm is not None else "无（回退 Python）"}')
    if m.nm is None:
        return 2

    g0 = cv2.cvtColor(grab_img(FRAMES[0], W, H), cv2.COLOR_BGR2GRAY)
    m.detect(g0, det='st')       # 预热

    same, thw, thp, tow, top = [], [], [], [], []
    for f in FRAMES:
        g = cv2.cvtColor(grab_img(f, W, H), cv2.COLOR_BGR2GRAY)

        m.nms = 'on'
        phw, n_hw, _ = m.detect(g, det='st')
        t_hw, thr_hw = m.t['nms'], m._thr_used

        m.nms = 'off'
        pyp, n_py, _ = m.detect(g, det='st')
        t_py, thr_py = m.t['select'], m._thr_used
        m.nms = 'auto'

        eq = (len(phw) == len(pyp) and np.array_equal(phw, pyp))
        same.append(eq)
        thw.append(t_hw)
        thp.append(t_py)
        c = cv2.goodFeaturesToTrack(g, MAXCNT, 0.01, MIND)
        c = np.zeros((0, 2)) if c is None else c.reshape(-1, 2)
        ow = rep(phw.astype(np.float64), c.astype(np.float64))
        op = rep(pyp.astype(np.float64), c.astype(np.float64))
        tow.append(ow)
        top.append(op)
        log(f'  帧 {f:3d}: 输出 {n_hw:3d} 点 阈值 {thr_hw} | 硬件 {len(phw):2d} 点 {t_hw:5.2f} ms | '
            f'Python {len(pyp):2d} 点 {t_py:5.2f} ms | '
            f'{"逐点逐序相同" if eq else "★不一致"} | 对 cv2 重合 {ow*100:.0f}% / {op*100:.0f}%')

    log(f'\n一致 {sum(same)}/{len(same)} 帧')
    log(f'选点耗时：硬件 {np.median(thw):.2f} ms（含 max 读取） vs Python {np.median(thp):.2f} ms '
        f'⇒ 省 {np.median(thp) - np.median(thw):.2f} ms')
    log(f'对 cv2 点集重合：硬件 {np.mean(tow)*100:.1f}% / Python {np.mean(top)*100:.1f}%')
    return 0 if all(same) else 1


if __name__ == '__main__':
    sys.exit(main())
