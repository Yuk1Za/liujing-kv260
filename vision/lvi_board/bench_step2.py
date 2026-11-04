#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拆解：① grab_img 三段（imread / resize / letterbox）
        ② PnPVO.step 三段（assign_depth / solvePnPRansac / pnp_covariance），
           分别喂 cv2 匹配与 PL 匹配，验证「step 差异 = RANSAC 自适应迭代次数」
"""
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, '/home/ubuntu/lvi_board')
from vo_core import assign_depth, pnp_covariance, rigid, inv

IMG = '/home/ubuntu/kitti_cam/image_02/data'
VO = '/home/ubuntu/_kitti_incoming/velodyne_points/data'
CAL = '/home/ubuntu/kiss_icp_fpga/evidence/reference_raw_v3'


def t_imread(k):
    t0 = time.perf_counter()
    im = cv2.imread(f'{IMG}/{k:010d}.png')
    return im, time.perf_counter() - t0


def t_resize(im):
    t0 = time.perf_counter()
    nh = int(round(im.shape[0] * 640 / im.shape[1]))
    r = cv2.resize(im, (640, nh), interpolation=cv2.INTER_AREA)
    return r, time.perf_counter() - t0


def t_letter(r):
    t0 = time.perf_counter()
    out = np.zeros((480, 640, 3), np.uint8)
    top = (480 - r.shape[0]) // 2
    out[top:top + r.shape[0]] = r
    return out, time.perf_counter() - t0


a, b, c = [], [], []
for k in range(12):
    im, d1 = t_imread(k)
    r, d2 = t_resize(im)
    _, d3 = t_letter(r)
    a.append(d1 * 1e3)
    b.append(d2 * 1e3)
    c.append(d3 * 1e3)
print(f'① grab_img 拆解(ms): imread {np.median(a):.2f} + resize {np.median(b):.2f} + letterbox {np.median(c):.2f} '
      f'= {np.median(a)+np.median(b)+np.median(c):.2f}')


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
T_v_c0 = rigid(cc['R_rect_00'].reshape(3, 3), np.zeros(3)) @ rigid(vc['R'].reshape(3, 3), vc['T'].ravel())
T_v_c2 = rigid(np.eye(3), -t_off) @ T_v_c0
K = np.array([[369.1176508972267, 0, 314.1987601957585],
              [0, 369.1176508972267, 238.58760195758566], [0, 0, 1]])
print(f'   fx={K[0,0]:.2f}')


def lidar(k):
    x = np.fromfile(f'{VO}/{k:010d}.bin', np.float32).reshape(-1, 4)[::2, :3].astype(np.float64)
    C = x @ T_v_c2[:3, :3].T + T_v_c2[:3, 3]
    C = C[C[:, 2] > 1.0]
    u = K[0, 0] * C[:, 0] / C[:, 2] + K[0, 2]
    v = K[1, 1] * C[:, 1] / C[:, 2] + K[1, 2]
    m = (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
    return np.stack([u[m], v[m]], 1), C[m, 2]


def gray(k):
    im = cv2.imread(f'{IMG}/{k:010d}.png')
    nh = int(round(im.shape[0] * 640 / im.shape[1]))
    r = cv2.resize(im, (640, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((480, 640, 3), np.uint8)
    out[(480 - nh) // 2:(480 - nh) // 2 + nh] = r
    return cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)


from pl_match import PLMatcher
m = PLMatcher(levels=3, max_iter=5)

acc = {}
for k in (3, 19, 63, 131):
    g1, g2 = gray(k), gray(k + 1)
    lu, lz = lidar(k)
    p0 = cv2.goodFeaturesToTrack(g1, 60, 0.05, 20.0).reshape(-1, 2)
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0.reshape(-1, 1, 2).astype(np.float32), None,
                                        winSize=(21, 21), maxLevel=3)
    st = st.reshape(-1).astype(bool)
    r = m.match(g1, g2)
    sets = {'cv2': (p0[st], p1.reshape(-1, 2)[st]), 'pl': (r['uv_prev'], r['uv_cur'])}
    for name, (up, uc) in sets.items():
        t0 = time.perf_counter()
        X3, good = assign_depth(up, lu, lz, K)
        t1 = time.perf_counter()
        uv = uc[good]
        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            X3.reshape(-1, 1, 3), uv.reshape(-1, 1, 2), K, None,
            flags=cv2.SOLVEPNP_EPNP, reprojectionError=2.0, iterationsCount=100, confidence=0.99)
        t2 = time.perf_counter()
        if ok and inl is not None and len(inl) >= 10:
            idx = inl.reshape(-1)
            R, _ = cv2.Rodrigues(rvec)
            Tb = inv(rigid(R, tvec.reshape(3)))
            pnp_covariance(X3[idx], uv[idx], Tb, K)
        t3 = time.perf_counter()
        acc.setdefault(name, []).append(
            ((t1 - t0) * 1e3, (t2 - t1) * 1e3, (t3 - t2) * 1e3,
             len(X3), 0 if inl is None else len(inl)))

print('② step 拆解(ms, 中位)：')
for name, v in acc.items():
    a = np.array([[x[0], x[1], x[2]] for x in v])
    print(f'   {name:4s} 赋深 {np.median(a[:,0]):5.1f} + RANSAC {np.median(a[:,1]):5.1f} '
          f'+ cov {np.median(a[:,2]):4.1f} = {np.median(a.sum(1)):5.1f}   '
          f'赋深点 {int(np.median([x[3] for x in v]))} 内点 {int(np.median([x[4] for x in v]))}')
