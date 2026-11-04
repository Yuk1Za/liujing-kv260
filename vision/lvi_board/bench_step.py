#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补测：① grab_img（本地读 PNG + resize + letterbox）单帧耗时
        ② PnPVO.step 在「60 个 cv2 特征」与「60 个 PL 特征」上的耗时对比
           （排除调度噪声，确认 step 与前端无关）
"""
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, '/home/ubuntu/lvi_board')
from vo_core import PnPVO, rigid, inv

IMG = '/home/ubuntu/kitti_cam/image_02/data'


def grab(idx, wimg=640, himg=480):
    im = cv2.imread(f'{IMG}/{idx:010d}.png')
    nh = int(round(im.shape[0] * wimg / im.shape[1]))
    r = cv2.resize(im, (wimg, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((himg, wimg, 3), np.uint8)
    top = (himg - nh) // 2
    out[top:top + nh] = r
    return out


t = []
for k in range(20):
    t0 = time.perf_counter()
    grab(k)
    t.append(time.perf_counter() - t0)
print(f'① grab_img: 中位 {1000*np.median(t):.2f} ms  (p90 {1000*np.percentile(t,90):.2f})')

# ② step：同一对帧，两种前端给出的匹配各跑一遍
def gray(i):
    return cv2.cvtColor(grab(i), cv2.COLOR_BGR2GRAY)


CAL = '/home/ubuntu/kiss_icp_fpga/evidence/reference_raw_v3'


def load_calib(path):
    d = {}
    for line in open(path):
        if ':' in line:
            k, v = line.split(':', 1)
            try:
                d[k.strip()] = np.array([float(x) for x in v.split()])
            except ValueError:
                pass
    return d


vc = load_calib(f'{CAL}/calib_velo_to_cam.txt')
cc = load_calib(f'{CAL}/calib_cam_to_cam.txt')
P2 = cc['P_rect_02'].reshape(3, 4)
t_off = np.array([P2[0, 3] / P2[0, 0], P2[1, 3] / P2[1, 1], 0.0])
T_v_c0 = rigid(cc['R_rect_00'].reshape(3, 3), np.zeros(3)) @ \
         rigid(vc['R'].reshape(3, 3), vc['T'].ravel())
T_v_c2 = rigid(np.eye(3), -t_off) @ T_v_c0
K = np.array([[369.1176508972267, 0, 314.1987601957585],
              [0, 369.1176508972267, 238.58760195758566], [0, 0, 1]])

VO = '/home/ubuntu/_kitti_incoming/velodyne_points/data'


def lidar(k):
    x = np.fromfile(f'{VO}/{k:010d}.bin', np.float32).reshape(-1, 4)[::2, :3].astype(np.float64)
    C = x @ T_v_c2[:3, :3].T + T_v_c2[:3, 3]
    C = C[C[:, 2] > 1.0]
    u = K[0, 0] * C[:, 0] / C[:, 2] + K[0, 2]
    v = K[1, 1] * C[:, 1] / C[:, 2] + K[1, 2]
    m = (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
    return np.stack([u[m], v[m]], 1), C[m, 2]


from pl_match import PLMatcher

m = PLMatcher(levels=3, max_iter=5)
res = {'cv2': [], 'pl': []}
for k in (3, 19, 63, 131):
    g1, g2 = gray(k), gray(k + 1)
    lu, lz = lidar(k)
    p0 = cv2.goodFeaturesToTrack(g1, 60, 0.05, 20.0).reshape(-1, 2)
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0.reshape(-1, 1, 2).astype(np.float32), None,
                                        winSize=(21, 21), maxLevel=3)
    st = st.reshape(-1).astype(bool)
    sets = {'cv2': (p0[st], p1.reshape(-1, 2)[st])}
    r = m.match(g1, g2)
    sets['pl'] = (r['uv_prev'], r['uv_cur'])
    for name, (up, uc) in sets.items():
        for _ in range(2):
            vo = PnPVO(K)
            t0 = time.perf_counter()
            rr = vo.step(up, uc, lu, lz)
            dt = time.perf_counter() - t0
            if rr is not None:
                res[name].append((dt, rr['n_used'], rr['n_inliers'], rr['rms_px']))
for name, v in res.items():
    if not v:
        print(f'② {name}: 全部失败')
        continue
    d = np.array([x[0] for x in v])
    print(f'② {name}: step 中位 {1000*np.median(d):.1f} ms  '
          f'赋深 {int(np.median([x[1] for x in v]))} 内点 {int(np.median([x[2] for x in v]))} '
          f'rms {np.median([x[3] for x in v]):.2f}px  (n={len(v)})')
