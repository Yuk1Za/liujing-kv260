#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量板上 vo_pnp_node 的精度：**逐条相对位姿**（给 BetweenFactor 的量）。

dump 格式（每行）：`f_prev f_cur  <T_between 16>  <T_abs 16>`

为什么不看累积轨迹
------------------
降频（every>1）时节点只算 (k-1,k) 相邻对，中间帧没有位姿 ⇒ `vo.T = vo.T @ T_between`
的下标不连续（T_c0←c4 · T_{7←8}），累积位姿数学上无意义（实测会给出 37 m 的假误差）。
**真正喂给 BetweenFactor 的是 T_between，所以逐条比它才是有意义的指标。**
"""
import argparse
import os

import numpy as np

H = os.environ.get("LVI_HOME", "/home/ubuntu")
CAL = f"{H}/kiss_icp_fpga/evidence/reference_raw_v3"
GT = f"{H}/lvi_board/gt_pose_fixed.csv"


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dump', required=True)
    ap.add_argument('--abs', action='store_true',
                    help='同时评累积轨迹（只在 every=1 时有意义）')
    a = ap.parse_args()

    rows = [l.split() for l in open(a.dump) if l.strip()]
    if not rows:
        print('[err] dump 为空')
        return
    G = gt_cam2()
    dp, dr, tn, skipped = [], [], [], 0
    for r in rows:
        kp, kc = int(r[0]), int(r[1])
        Tb = np.array([float(x) for x in r[2:18]]).reshape(4, 4)
        if kc - kp != 1:
            skipped += 1
            continue
        # ★ inv(G[kp]) @ G[kc] 已经是 T_{kp←kc}（与 T_between 同向）。
        #   外面**再**套一层 inv 会得到 T_{kc←kp} ⇒ 平移方向反、差值 = 2|t|
        #   （实测报出 1.24 m，而真实帧间位移只有 0.614 m）。这是同一个"乘逆/不乘逆"陷阱。
        dTg = np.linalg.inv(G[kp]) @ G[kc]
        dp.append(np.linalg.norm(Tb[:3, 3] - dTg[:3, 3]))
        dr.append(rot_angle(Tb[:3, :3].T @ dTg[:3, :3]))
        tn.append(np.linalg.norm(dTg[:3, 3]))
    dp, dr, tn = map(np.array, (dp, dr, tn))
    print(f'[逐条] {len(dp)} 条相邻对（跳过 {skipped} 条非相邻）  '
          f'GT 帧间位移 中位 {np.median(tn):.3f} m  总 {tn.sum():.1f} m')
    print(f'[平移] 中位 {np.median(dp):.4f} m  p90 {np.percentile(dp,90):.4f}  max {dp.max():.4f}')
    print(f'[旋转] 中位 {np.median(dr):.3f}°  p90 {np.percentile(dr,90):.3f}  max {dr.max():.3f}')
    print(f'[相对误差] 平移 {100*np.median(dp/np.maximum(tn,1e-9)):.1f}%   '
          f'>2° 的条数 {int((dr>2).sum())}/{len(dr)}')

    if a.abs:
        Ta, fa = [], []
        for r in rows:
            Ta.append(np.array([float(x) for x in r[18:34]]).reshape(4, 4))
            fa.append(int(r[1]))
        # 只保留连续帧段（every=1 时全部连续）
        W0 = G[fa[0]] @ np.linalg.inv(Ta[0])
        ep = np.array([np.linalg.norm((W0 @ Ta[k])[:3, 3] - G[fa[k]][:3, 3])
                       for k in range(len(fa))])
        er = np.array([rot_angle((W0 @ Ta[k])[:3, :3].T @ G[fa[k]][:3, :3])
                       for k in range(len(fa))])
        span = np.linalg.norm(np.diff(np.array([G[f][:3, 3] for f in fa]), axis=0), axis=1).sum()
        print(f'[累积] {len(fa)} 帧，帧 {fa[0]}..{fa[-1]}，GT 行程 {span:.1f} m ⇒ '
              f'位置 RMS {np.sqrt((ep**2).mean()):.3f} m  max {ep.max():.3f}  |  '
              f'旋转 RMS {np.sqrt((er**2).mean()):.2f}°')


if __name__ == '__main__':
    main()
