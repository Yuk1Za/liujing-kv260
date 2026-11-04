#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chk_lkv2.py —— 新位流（lk_lvisam, Q4）加载后的 1 分钟功能校验。

判据：
  · PL 跟踪点数应接近 cv2 的成功数（不是 0、也不是全灭）
  · PL 流 vs cv2 流（**同一批点**）误差中位应在 **0.1–0.3 px**；
    若看到 ~4× 关系 ⇒ Q 没切（用了 Q6 去解 Q4 的输出）；
    若看到 ~2× 或恒定偏差 ⇒ 坐标系/方向问题，别怀疑算法。
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, '/home/ubuntu/lvi_board')
IMG = '/home/ubuntu/kitti_cam/image_02/data'
BIT = os.environ.get('PL_BIT', '/home/ubuntu/vis3/design_1_wrapper_lkv2.bit')


def gray(i):
    im = cv2.imread(f'{IMG}/{i:010d}.png')
    nh = int(round(im.shape[0] * 640 / im.shape[1]))
    r = cv2.resize(im, (640, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((480, 640, 3), np.uint8)
    out[(480 - nh) // 2:(480 - nh) // 2 + nh] = r
    return cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)


from pl_match import PLMatcher

print(f'位流 {BIT}  Q={os.environ.get("PL_Q", 4)}')
m = PLMatcher(bit=BIT, levels=3, max_iter=30, max_cnt=60, q=int(os.environ.get('PL_Q', 4)))
m.match(gray(3), gray(3))          # warmup

BUCK = [(0, 2), (2, 5), (5, 10), (10, 20), (20, 1e9)]
err = {b: [] for b in BUCK}
tot = [0, 0]
for k in (3, 11, 19, 63, 131):
    g1, g2 = gray(k), gray(k + 1)
    r = m.match(g1, g2)
    up, uc = r['uv_prev'], r['uv_cur']
    if len(up) == 0:
        print(f'  帧{k}→{k+1}: **PL 0 点** ⇒ 位流/接口不对')
        continue
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, up.reshape(-1, 1, 2).astype(np.float32), None,
                                        winSize=(21, 21), maxLevel=3)
    okc = st.reshape(-1).astype(bool)
    fp, fc = uc - up, p1.reshape(-1, 2) - up
    e = np.linalg.norm(fp[okc] - fc[okc], axis=1)
    mag = np.linalg.norm(fc[okc], axis=1)
    for b in BUCK:
        sel = (mag >= b[0]) & (mag < b[1])
        err[b].extend(e[sel].tolist())
    tot[0] += int(okc.sum())
    tot[1] += len(up)
    print(f'  帧{k}→{k+1}: PL {len(up)} 点（cv2 同点成功 {int(okc.sum())}）  '
          f'误差中位 {np.median(e):.3f} px  p90 {np.percentile(e,90):.3f}  '
          f'| PL 匹配耗时 {r["t"]["total"]:.2f} ms（LK {r["t"]["lk"]:.2f}）')

print()
alle = np.concatenate([np.array(v) for v in err.values() if v])
print(f'合计：PL 点 {tot[1]}，与 cv2 共同成功 {tot[0]}，误差中位 {np.median(alle):.3f} px')
for b in BUCK:
    v = np.array(err[b])
    if v.size == 0:
        continue
    print(f'  [{b[0]},{b[1] if b[1] < 1e8 else "∞"}) n={v.size:3d} '
          f'中位 {np.median(v):7.3f}  <1px {100*np.mean(v < 1):5.1f}%')
