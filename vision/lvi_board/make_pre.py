#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_pre.py —— 把 KITTI 原图**离线**转成节点实际看到的灰度图（640×480，letterbox 后）。

为什么要它：节点每次唤醒要读 2 张图，板上实测 **105 ms/唤醒**（imread 33.9 + resize 18.5，各 ×2），
占整帧的 50%，比视觉匹配本身还贵。既然分辨率/letterbox 是固定的，
就把「解码 + 缩放 + 补边 + 灰度」全部前移到离线，在线只做一次小图解码。

★ 必须**逐位一致**：这里复刻的就是 `vo_pnp_node.grab_img` + `cvtColor` 的顺序
  （BGR 原图 → INTER_AREA 缩到 640 宽 → 零填充到 480 高 → 转灰度），
  所以替换后进 PL/PnP 的像素与原来**完全相同**，精度不应有任何变化（可选 `--verify` 校验）。

用法（板端，不需要 root）：
  python3 make_pre.py 0 600            # 生成 0..599
  python3 make_pre.py 0 600 --verify   # 顺便逐位比对
"""
import os
import sys
import time

import numpy as np
import cv2

SRC = '/home/ubuntu/kitti_cam/image_02/data'
DST = os.environ.get('PRE_DIR', '/home/ubuntu/kitti_pre')
W, H = 640, 480


def make(idx):
    im = cv2.imread(f'{SRC}/{idx:010d}.png')
    if im is None:
        return None
    nh = int(round(im.shape[0] * W / im.shape[1]))
    r = cv2.resize(im, (W, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((H, W, 3), np.uint8)
    top = (H - nh) // 2
    out[top:top + nh] = r
    return cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)


def main():
    a, b = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (0, 600)
    verify = '--verify' in sys.argv
    os.makedirs(DST, exist_ok=True)
    ok = miss = 0
    t0 = time.perf_counter()
    for i in range(a, b):
        g = make(i)
        if g is None:
            miss += 1
            continue
        cv2.imwrite(f'{DST}/{i:010d}.png', g, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if verify:
            back = cv2.imread(f'{DST}/{i:010d}.png', cv2.IMREAD_GRAYSCALE)
            if back is None or not np.array_equal(back, g):
                print(f'★ 第 {i} 帧不一致')
                return 1
        ok += 1
    dt = time.perf_counter() - t0
    print(f'生成 {ok} 张（缺 {miss}），{dt:.1f} s → {DST}')
    if verify:
        print('逐位校验：全部通过')
    # 抽查读图成本
    if ok:
        for tag, path, flag in (('原图 1226×370 彩色', f'{SRC}/{a:010d}.png', cv2.IMREAD_COLOR),
                                ('预缩放 640×480 灰度', f'{DST}/{a:010d}.png', cv2.IMREAD_GRAYSCALE)):
            t0 = time.perf_counter()
            for _ in range(10):
                cv2.imread(path, flag)
            print(f'  {tag}: {(time.perf_counter()-t0)*100:.1f} ms/张')
    return 0


if __name__ == '__main__':
    sys.exit(main())
