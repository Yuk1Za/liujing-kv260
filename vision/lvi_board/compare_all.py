#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把多份 dump 放到**同一批配对**上比较（消除"成功对数不同"带来的中位数偏差）。
用法: python3 compare_all.py traj_base200.txt traj_hyb.txt traj_pl3.txt traj_pl4.txt
"""
import os
import sys

import numpy as np

H = os.environ.get('LVI_HOME', '/home/ubuntu')
CAL = f'{H}/kiss_icp_fpga/evidence/reference_raw_v3'
GT = f'{H}/lvi_board/gt_pose_fixed.csv'


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


def rigid(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(t).reshape(3)
    return M


def gt_cam2():
    vc = load_calib(f'{CAL}/calib_velo_to_cam.txt')
    cc = load_calib(f'{CAL}/calib_cam_to_cam.txt')
    P2 = cc['P_rect_02'].reshape(3, 4)
    t_off = np.array([P2[0, 3] / P2[0, 0], P2[1, 3] / P2[1, 1], 0.0])
    T_v_c0 = rigid(cc['R_rect_00'].reshape(3, 3), np.zeros(3)) @ \
             rigid(vc['R'].reshape(3, 3), vc['T'].ravel())
    g = np.loadtxt(GT, delimiter=',', skiprows=1)
    out = []
    for i in range(len(g)):
        w, x, y, z = g[i, 9], g[i, 6], g[i, 7], g[i, 8]
        n = np.sqrt(w * w + x * x + y * y + z * z)
        w, x, y, z = w / n, x / n, y / n, z / n
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        out.append(rigid(R, g[i, 3:6]) @ np.linalg.inv(T_v_c0) @ rigid(np.eye(3), t_off))
    return out


def rot_angle(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


G = gt_cam2()
runs = {}
for f in sys.argv[1:]:
    d = {}
    for ln in open(f):
        r = ln.split()
        if not r:
            continue
        kp, kc = int(r[0]), int(r[1])
        if kc - kp != 1:
            continue
        d[kc] = np.array([float(x) for x in r[2:18]]).reshape(4, 4)
    runs[os.path.basename(f)] = d
    print(f'{os.path.basename(f)}: {len(d)} 条')

common = sorted(set.intersection(*[set(v) for v in runs.values()]))
print(f'\n共同配对 {len(common)} 条（帧 {common[0]}..{common[-1]}）\n')

hdr = f'{"":16s}{"平移中位":>11s}{"p90":>9s}{"max":>9s}{"旋转中位":>10s}{"旋转p90":>9s}{">2°":>7s}{"相对误差":>10s}'
print(hdr)
print('-' * len(hdr))
for name, d in runs.items():
    dp, dr, tn = [], [], []
    for kc in common:
        Tb = d[kc]
        dTg = np.linalg.inv(G[kc - 1]) @ G[kc]
        dp.append(np.linalg.norm(Tb[:3, 3] - dTg[:3, 3]))
        dr.append(rot_angle(Tb[:3, :3].T @ dTg[:3, :3]))
        tn.append(np.linalg.norm(dTg[:3, 3]))
    dp, dr, tn = map(np.array, (dp, dr, tn))
    print(f'{name[:16]:16s}{np.median(dp):11.4f}{np.percentile(dp,90):9.4f}{dp.max():9.4f}'
          f'{np.median(dr):10.3f}{np.percentile(dr,90):9.3f}{int((dr > 2).sum()):7d}'
          f'{100*np.median(dp/np.maximum(tn,1e-9)):9.1f}%')
    if name == list(runs)[0]:
        print(f'{"（GT 帧间位移中位）":16s}{np.median(tn):11.3f} m   总 {tn.sum():.1f} m')
