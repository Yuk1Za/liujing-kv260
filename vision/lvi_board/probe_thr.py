#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_thr.py —— 固定阈值 thr=1 与「自适应 0.01×max」的点集是否一致（决定能否省掉 max 读取）。"""
import sys, time
import numpy as np, cv2
sys.path.insert(0,'/home/ubuntu/lvi_board')
from pl_match import PLMatcher, BIT
from vo_pnp_node import grab_img
m = PLMatcher(bit=BIT, max_cnt=60, min_dist=20, levels=2, max_iter=8, nms='on')
g0 = cv2.cvtColor(grab_img(0,640,480), cv2.COLOR_BGR2GRAY); m.detect(g0, det='st')
same=0; tot=0; t1s=[]; t2s=[]
for f in (0,60,120,180,240,300,420,540):
    g = cv2.cvtColor(grab_img(f,640,480), cv2.COLOR_BGR2GRAY)
    m.nms='on'
    p1,_,_ = m.detect(g, det='st', th=1); t1=m.t['nms']; s1=m._thr_used
    p2,_,_ = m.detect(g, det='st', th=0); t2=m.t['nms']; s2=m._thr_used
    eq = (len(p1)==len(p2) and np.array_equal(p1,p2))
    same += eq; tot += 1; t1s.append(t1); t2s.append(t2)
    print(f'  帧 {f:3d}: thr=1 → {len(p1)} 点 {t1:.2f} ms | 自适应 thr={s2} → {len(p2)} 点 {t2:.2f} ms | {"相同" if eq else "★不同"}')
print(f'\n一致 {same}/{tot}   thr=1 中位 {np.median(t1s):.2f} ms vs 自适应 {np.median(t2s):.2f} ms')
