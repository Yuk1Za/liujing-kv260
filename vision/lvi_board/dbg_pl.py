#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""调试：逐层打点，看 it=5 为什么全灭。"""
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


g1, g2 = gray(3), gray(4)
p0 = cv2.goodFeaturesToTrack(g1, 60, 0.05, 20.0).reshape(-1, 2)
p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0.reshape(-1, 1, 2).astype(np.float32), None,
                                     winSize=(21, 21), maxLevel=3)
ref = p1.reshape(-1, 2) - p0
print('cv2 参考流 中位模 %.2f px  max %.2f' % (np.median(np.linalg.norm(ref, axis=1)),
                                            np.linalg.norm(ref, axis=1).max()))

from pl_match import PLMatcher

for it in (5, 10, 20):
    print(f'--- max_iter={it} ---')
    m = PLMatcher(levels=4, max_iter=it)
    orig = m._run_lk

    def wrap(l, pts, flow, n, _o=orig):
        out, tr = _o(l, pts, flow, n)
        med = ''
        if tr.sum():
            med = f'流中位 u={np.median(out[tr,0]):7.2f} v={np.median(out[tr,1]):6.2f}'
        print(f'   L{l} n={n:2d} tracked={int(tr.sum()):2d} {med} '
              f'|u|max={np.abs(out[tr,0]).max() if tr.sum() else 0:6.1f}')
        return out, tr
    m._run_lk = wrap
    r = m.match(g1, g2)
    print('   total %.2f ms  tracked %d  t=%s' % (
        r['t']['total'], len(r['uv_prev']),
        {k: round(v, 2) for k, v in r['t'].items()}))
