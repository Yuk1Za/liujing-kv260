#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""板上 vo_core 基准：分项计时 + （有 GT 时）精度。

为什么要分项
------------
服务器上 `solvePnPRansac` 只要 0.25 ms，但 `assign_depth` 是纯 Python 循环
（每个特征都要对整帧 12 万激光点算一次距离）——在 4 核 ARM 上它可能比 PnP 贵两个数量级。
不分项就会把"赋深慢"误判成"PnP 慢"，进而去优化错的东西。

板上与服务器的已知差异
--------------------
* cv2 **4.5.4**（服务器是 5.0.0）：`solvePnPRansac` 的 inliers 两边都是索引数组，语义一致；
  但此处仍做长度校验，避免版本差异静默给出错位结果。
* 分辨率按 **640×480**（板端 PL 的固定输出），内参随之换算；这与定版 720×540 不同，
  所以精度数字不能直接和服务器 10/10 那次比。
"""
import argparse
import json
import os
import time

import numpy as np
import cv2

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vo_core import PnPVO, assign_depth, pnp_covariance, rigid, inv

HOME = os.path.expanduser('~')
# 路径可用环境变量覆盖，这样同一份脚本能在服务器（验证）与板上（实测）都用
IMG = os.environ.get('VO_IMG', f'{HOME}/kitti_cam/image_02/data')
VEL = os.environ.get('VO_VEL', f'{HOME}/_kitti_incoming/velodyne_points/data')
CAL = os.environ.get('VO_CAL', f'{HOME}/kiss_icp_fpga/evidence/reference_raw_v3')
GT = os.environ.get('VO_GT', f'{HOME}/lvi_board/gt_pose_fixed.csv')


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


def build_K(wimg, himg):
    """与 pl_vision_node._grab() 完全一致的 letterbox 内参。"""
    scale = wimg / 1226.0
    nh = int(round(370 * scale))
    top = (himg - nh) // 2
    fx = 707.091 * scale
    cx = 601.887 * scale
    cy = 183.110 * scale + top
    K = np.array([[fx, 0, cx], [0, fx, cy], [0, 0, 1]], float)
    return K, scale, nh, top


def calib():
    vc = load_calib(f'{CAL}/calib_velo_to_cam.txt')
    cc = load_calib(f'{CAL}/calib_cam_to_cam.txt')
    P2 = cc['P_rect_02'].reshape(3, 4)
    t_off = np.array([P2[0, 3] / P2[0, 0], P2[1, 3] / P2[1, 1], 0.0])
    T_v_c0 = rigid(cc['R_rect_00'].reshape(3, 3), np.zeros(3)) @ \
             rigid(vc['R'].reshape(3, 3), vc['T'].ravel())
    return T_v_c0, rigid(np.eye(3), -t_off) @ T_v_c0, rigid(np.eye(3), t_off)


def grab(idx, wimg, himg):
    im = cv2.imread(f'{IMG}/{idx:010d}.png')
    if im is None:
        return None
    sc = wimg / im.shape[1]
    nh = int(round(im.shape[0] * sc))
    r = cv2.resize(im, (wimg, nh), interpolation=cv2.INTER_AREA)
    top = (himg - nh) // 2
    out = np.zeros((himg, wimg, 3), np.uint8)
    out[top:top + nh] = r
    return out


def load_velo(idx):
    return np.fromfile(f'{VEL}/{idx:010d}.bin', dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


def project_lidar(raw, T_v_c2, K, wimg, himg):
    """点云 → 图像坐标 + 深度（真实系统里点云已由 ROS 解码，故与读盘分开计时）。

    用 raw @ Rᵀ + t 代替 (T @ [x,1]) —— 数学等价，但少一次 hstack 与 4×4 乘。
    12 万点在板上这一步从 48.8 ms 降到约 30 ms（仍是第二大项，见 --stride 抽稀）。
    """
    C = raw @ T_v_c2[:3, :3].T + T_v_c2[:3, 3]
    m = C[:, 2] > 1.0
    C = C[m]
    u = K[0, 0] * C[:, 0] / C[:, 2] + K[0, 2]
    v = K[1, 1] * C[:, 1] / C[:, 2] + K[1, 2]
    ok = (u >= 0) & (u < wimg) & (v >= 0) & (v < himg)
    return np.stack([u[ok], v[ok]], 1), C[ok, 2]


def match(img_prev, img_cur, max_cnt, min_dist, win, lvl):
    """cv2 现算匹配（只为喂 vo_core；板上真实前端是 PL，已单独实测 8–15 ms）。"""
    g1 = cv2.cvtColor(img_prev, cv2.COLOR_BGR2GRAY)
    g2 = cv2.cvtColor(img_cur, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(g1, max_cnt, 0.05, min_dist)
    if p0 is None:
        return None, None
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0, None,
                                         winSize=(win, win), maxLevel=lvl)
    st = st.reshape(-1).astype(bool)
    return p0.reshape(-1, 2)[st], p1.reshape(-1, 2)[st]


def gt_cams(f0, W, T_v_c0, T_c0_c2):
    g = np.loadtxt(GT, delimiter=',', skiprows=1)
    M = inv(T_v_c0)
    out = []
    for i in range(W):
        w, x, y, z = g[f0 + i, 9], g[f0 + i, 6], g[f0 + i, 7], g[f0 + i, 8]
        n = np.sqrt(w * w + x * x + y * y + z * z)
        w, x, y, z = w / n, x / n, y / n, z / n
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        out.append(rigid(R, g[f0 + i, 3:6]) @ M @ T_c0_c2)
    return out


def rot_angle(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--F0', type=int, default=0)
    ap.add_argument('--W', type=int, default=60)
    ap.add_argument('--wimg', type=int, default=640)
    ap.add_argument('--himg', type=int, default=480)
    ap.add_argument('--max-cnt', type=int, default=150)
    ap.add_argument('--min-dist', type=float, default=20.0)
    ap.add_argument('--win', type=int, default=21)
    ap.add_argument('--lvl', type=int, default=3)
    ap.add_argument('--preload', action='store_true',
                    help='预加载点云到内存。真实系统点云来自 ROS 消息（已解码），'
                         '不该把 eMMC 读盘（板上实测 130 ms/帧）算进视觉开销')
    ap.add_argument('--stride', type=int, default=1, help='点云抽稀步长（1=不抽；>1 可换更小的投影开销）')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    K, scale, nh, top = build_K(args.wimg, args.himg)
    T_v_c0, T_v_c2, T_c0_c2 = calib()
    print(f'[cfg] {args.wimg}x{args.himg} letterbox: fx={K[0,0]:.3f} cx={K[0,2]:.3f} cy={K[1,2]:.3f} '
          f'(scale={scale:.4f}, nh={nh}, top={top})')
    print(f'[cfg] cv2 {cv2.__version__} / numpy {np.__version__} / 路标点云 {VEL}')

    vo = PnPVO(K)
    t = dict(grab=0.0, lidar_io=0.0, lidar=0.0, match=0.0, step=0.0, depth=0.0, pnp=0.0, cov=0.0)
    est = [vo.pose()]

    cache = {}
    if args.preload:
        tp = time.perf_counter()
        for kk in range(max(0, args.F0 - 1), args.F0 + args.W):
            cache[kk] = load_velo(kk)
        mb = sum(v.size for v in cache.values()) * 8 / 1e6
        print(f'[preload] {len(cache)} 帧点云 → 内存 {mb:.0f} MB，耗时 {time.perf_counter()-tp:.1f} s '
              f'(不计入每帧耗时)')

    has_gt = os.path.exists(GT)
    if not has_gt:
        print('[warn] 无 GT，跳过精度统计（只量耗时）')

    prev = None
    for k in range(args.W):
        idx = args.F0 + k
        t0 = time.perf_counter()
        img = grab(idx, args.wimg, args.himg)
        t['grab'] += time.perf_counter() - t0
        if img is None:
            print(f'[err] 读不到图 {idx}')
            break
        if k == 0:
            prev = img
            continue

        t0 = time.perf_counter()
        raw = cache[idx - 1] if (idx - 1) in cache else load_velo(idx - 1)
        if args.stride > 1:
            raw = raw[::args.stride]
        t['lidar_io'] += time.perf_counter() - t0
        t0 = time.perf_counter()
        lu, lz = project_lidar(raw, T_v_c2, K, args.wimg, args.himg)
        t['lidar'] += time.perf_counter() - t0

        t0 = time.perf_counter()
        up, uc = match(prev, img, args.max_cnt, args.min_dist, args.win, args.lvl)
        t['match'] += time.perf_counter() - t0
        if up is None or len(up) < 8:
            prev = img
            est.append(vo.pose())
            continue

        # ---- ① step 整体（真实调用路径）----
        t0 = time.perf_counter()
        r = vo.step(up, uc, lu, lz)
        t['step'] += time.perf_counter() - t0
        est.append(vo.pose())

        # ---- ② 分项（同样输入再算一遍，仅用于归因；不影响上面的 step 计时）----
        t0 = time.perf_counter()
        X3, good = assign_depth(up, lu, lz, K)
        t['depth'] += time.perf_counter() - t0
        if r is not None and len(X3) >= 8:
            t0 = time.perf_counter()
            cv2.solvePnPRansac(X3.reshape(-1, 1, 3), uc[good].reshape(-1, 1, 2), K, None,
                               flags=cv2.SOLVEPNP_EPNP, reprojectionError=2.0,
                               iterationsCount=100, confidence=0.99)
            t['pnp'] += time.perf_counter() - t0
            t0 = time.perf_counter()
            pnp_covariance(X3[:40], uc[good][:40], r['T_between'], K)
            t['cov'] += time.perf_counter() - t0
        prev = img

        if k % 20 == 0:
            print(f'  f{idx} 匹配 {len(up):3d} 赋深 {len(X3):3d} '
                  f'{"内点 %d rms %.2f px |t|=%.3f" % (r["n_inliers"], r["rms_px"], r["t_norm"]) if r else "step=None"}',
                  flush=True)

    n = max(args.W - 1, 1)
    print(f'\n[耗时] 每帧均值（{args.W} 帧，{vo.n_ok} 次 step）')
    for kk, vv in t.items():
        print(f'  {kk:6s} {1000*vv/n:8.2f} ms/帧')
    print(f'  [汇总] step 实测 {1000*t["step"]/n:.2f} ms/帧 ；'
          f'分项相加 {1000*(t["depth"]+t["pnp"]+t["cov"])/n:.2f} ms/帧（赋深 {1000*t["depth"]/n:.2f} + '
          f'PnP {1000*t["pnp"]/n:.2f} + cov {1000*t["cov"]/n:.2f}）')
    print(f'[统计] step 成功 {vo.n_ok} / 拒绝 {vo.n_reject}')

    if has_gt:
        G = gt_cams(args.F0, len(est), T_v_c0, T_c0_c2)
        We = [G[0] @ est[k] for k in range(len(est))]
        pos = np.array([np.linalg.norm(We[k][:3, 3] - G[k][:3, 3]) for k in range(len(est))])
        rot = np.array([rot_angle(We[k][:3, :3].T @ G[k][:3, :3]) for k in range(len(est))])
        print(f'[精度] 位置 RMS {np.sqrt((pos**2).mean()):.3f} m  max {pos.max():.3f}  |  '
              f'旋转 RMS {np.sqrt((rot**2).mean()):.2f}°  max {rot.max():.2f}°')
    else:
        pos = rot = np.zeros(1)

    res = dict(cfg=vars(args), K=K.tolist(), cv2=cv2.__version__,
               ms_per_frame={k2: 1000 * v2 / n for k2, v2 in t.items()},
               n_ok=vo.n_ok, n_reject=vo.n_reject,
               pos_rms=float(np.sqrt((pos**2).mean())), rot_rms=float(np.sqrt((rot**2).mean())))
    out = args.out or f'{HOME}/lvi_board/bench_vo.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(res, open(out, 'w'), ensure_ascii=False, indent=1)
    print(f'[out] {out}')


if __name__ == '__main__':
    main()
