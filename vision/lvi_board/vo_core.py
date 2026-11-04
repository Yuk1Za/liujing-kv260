#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vo_core —— 单目 + 激光赋深的 PnP 视觉里程计核心（无文件 IO，可直接被 ROS2 节点调用）。

设计要点
--------
* 只依赖 OpenCV + numpy（板上已有；**不需要 Ceres**）。
* 输入是「相邻两帧的匹配特征」+「上一帧的激光点投影」，输出是**相对位姿 + 6×6 协方差**，
  正好是 `map_optimization` 里 `BetweenFactor<Pose3>` 需要的形式。
* 协方差用**数值雅可比**在外层输出参数化上直接算，避免换基/伴随矩阵推错：
  残差定义为 r = 投影(T_out⁻¹ 施加于 X) − uv，扰动 δ 作用在**第 k-1 帧**（左扰动，旋转在前）。

对外接口
--------
    vo = PnPVO(K, ...)
    r  = vo.step(uv_prev, uv_cur, lidar_uv, lidar_z)   # r 可能为 None（本帧不可用）
    vo.pose()            # 链式的 T_c0←ck（4x4）
    r['T_between']       # 4x4，T_{k-1←k}（给 BetweenFactor 的 measured）
    r['cov6']            # 6x6，顺序 (rx,ry,rz,tx,ty,tz)，与 GTSAM Pose3 切空间一致
"""
import numpy as np
import cv2

__all__ = ['PnPVO', 'assign_depth', 'pnp_covariance', 'rigid', 'inv']

# ------------------------------- 小工具 -----------------------------------


def rigid(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(t).reshape(3)
    return M


def inv(T):
    R = T[:3, :3]
    return rigid(R.T, -R.T @ T[:3, 3])


def _exp6(d):
    """旋转在前的 6 维指数映射。"""
    w = d[:3]
    th = np.linalg.norm(w)
    if th < 1e-12:
        Rd = np.eye(3) + np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    else:
        k = w / th
        Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        Rd = np.eye(3) + np.sin(th) * Kx + (1 - np.cos(th)) * (Kx @ Kx)
    return rigid(Rd, d[3:])


# --------------------------- 激光赋深 -------------------------------------


def assign_depth(uv_prev, lidar_uv, lidar_z, K, rad=4.0, rel=0.10, ab=0.20, min_pts=2):
    """给上一帧的每个特征赋深：搜索半径内取中位 + 一致性门控（挡跨深度跳变面）。

    返回 (X3, good)：X3 在**上一帧相机系**，只返回 good=True 的行。

    为什么按 u 预排序
    ----------------
    朴素写法对每个特征都算一次「到全帧激光点的距离」= O(n_feat × n_lidar)。
    KITTI 每帧 12.2 万点、约 70 个特征 ⇒ 840 万次运算，板上实测这项独占 `step` 的 **91%**
    （109 ms / 120 ms，服务器只 2.5 ms —— ARM 上 numpy 的标量循环放大 44×）。
    按 u 排序后每个特征只在 [cu−rad, cu+rad] 的切片里找（约 1.5% 的点），
    筛选条件与原实现完全相同，所以**结果逐位一致**（已用 vo_core_eval 的等价性测试复核）。
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    n = len(uv_prev)
    out = np.zeros((n, 3))
    good = np.zeros(n, bool)
    if len(lidar_uv) == 0 or n == 0:
        return out[good], good

    order = np.argsort(lidar_uv[:, 0], kind='stable')
    lu = lidar_uv[order]
    lz = lidar_z[order]
    us = lu[:, 0]
    r2 = rad * rad
    for i in range(n):
        cu, cv_ = uv_prev[i]
        lo = np.searchsorted(us, cu - rad, 'left')
        hi = np.searchsorted(us, cu + rad, 'right')
        if hi - lo < min_pts:
            continue
        du = lu[lo:hi, 0] - cu
        dv = lu[lo:hi, 1] - cv_
        sel = np.where(du * du + dv * dv <= r2)[0]
        if len(sel) < min_pts:
            continue
        zs = lz[lo:hi][sel]
        med = float(np.median(zs))
        keep = np.abs(zs - med) <= rel * med + ab
        if int(keep.sum()) < min_pts:
            continue
        z = float(np.median(zs[keep]))
        out[i] = [(cu - cx) / fx * z, (cv_ - cy) / fy * z, z]
        good[i] = True
    return out[good], good


# --------------------------- 协方差 ---------------------------------------


def _reproj(T_out, X3, uv, K):
    """用 T_out（T_{k-1←k}）把上一帧的 3D 点投到第 k 帧，返回 (2N,) 残差。"""
    R, t = T_out[:3, :3], T_out[:3, 3]
    Xk = (X3 - t) @ R                     # = (Rᵀ (X − t))ᵀ
    Z = Xk[:, 2]
    u = K[0, 0] * Xk[:, 0] / Z + K[0, 2]
    v = K[1, 1] * Xk[:, 1] / Z + K[1, 2]
    return np.concatenate([u - uv[:, 0], v - uv[:, 1]])


def pnp_covariance(X3, uv, T_between, K, h=1e-6):
    """数值雅可比求 6×6 协方差（顺序 rx,ry,rz,tx,ty,tz）。

    残差 r(δ) = 投影( (Exp(δ)·T_out)⁻¹ X ) − uv，δ 为第 k-1 帧下的左扰动。
    σ² 由内点残差估计：σ² = Σr² / (2N − 6)。
    """
    N = len(X3)
    if N < 8:
        return None
    r0 = _reproj(T_between, X3, uv, K)
    J = np.zeros((len(r0), 6))
    for j in range(6):
        d = np.zeros(6)
        d[j] = h
        Tp = _exp6(d) @ T_between
        Tm = _exp6(-d) @ T_between
        J[:, j] = (_reproj(Tp, X3, uv, K) - _reproj(Tm, X3, uv, K)) / (2 * h)
    H = J.T @ J
    dof = max(2 * N - 6, 1)
    sigma2 = float(r0 @ r0) / dof
    # 加一点对角正则，压住近奇异方向（例如纯前向运动时平移深度不可观）
    H = H + np.eye(6) * max(1e-9, np.trace(H) * 1e-12)
    try:
        cov = sigma2 * np.linalg.inv(H)
    except np.linalg.LinAlgError:
        return None
    cov = 0.5 * (cov + cov.T)
    if not np.all(np.isfinite(cov)) or np.any(np.diag(cov) <= 0):
        return None
    return cov


# --------------------------- 主类 -----------------------------------------


class PnPVO:
    def __init__(self, K, reproj_err=2.0, ransac_iters=100, conf=0.99,
                 min_inliers=10, min_pts=8, depth_rad=4.0,
                 depth_rel=0.10, depth_abs=0.20,
                 cov_scale=(1.6, 4.5)):
        # cov_scale：分块（旋转, 平移）标定系数，把"仅像素噪声"的解析协方差放大到真实误差量级。
        #   KITTI 0027、十段 80 m、1090 对拟合（使 1σ 覆盖率 = 68%）：rot 1.6×、trans 4.5×。
        #   平移欠估最严重（3.7×），因为**深度误差**不会在图像残差里体现，像素噪声模型看不到它。
        self.cov_scale = cov_scale
        self.K = np.asarray(K, float)
        self.reproj_err = reproj_err
        self.ransac_iters = ransac_iters
        self.conf = conf
        self.min_inliers = min_inliers
        self.min_pts = min_pts
        self.depth_rad, self.depth_rel, self.depth_abs = depth_rad, depth_rel, depth_abs
        self.T = np.eye(4)          # T_c0←ck（链式绝对位姿）
        self.n_ok = 0
        self.n_reject = 0

    # -- 单步：给一对匹配 + 上一帧激光投影，返回相对位姿与协方差 -------------
    def step(self, uv_prev, uv_cur, lidar_uv, lidar_z, X3_pre=None):
        """X3_pre：逐特征已算好的 3D 点（与 uv 一一对应）时**直喂 PnP**。

        为什么需要这条旁路（2026-10-27 diag_fail.py 实测）：RGB-D 赋深是逐特征
        一一对应的（每个 tracked 点查一次深度图），但把它当"激光点云"再过
        assign_depth 的 min_pts=2 邻域门控时，特征间距 ≥ min_dist(20 px) 而
        搜索半径只有 4 px ⇒ 每个特征邻域内只有它自己 1 个点 ⇒ 门控把全部点
        吃掉 ⇒ X3 恒空 ⇒ step 永远返回 None（A 路 Σ|X3|=0 / 128 对）。
        直喂旁路（C 路）128 对全部进 PnP，reproj=2 成功 85–111 对。
        """
        uv_prev = np.asarray(uv_prev, float).reshape(-1, 2)
        uv_cur = np.asarray(uv_cur, float).reshape(-1, 2)
        if len(uv_prev) != len(uv_cur) or len(uv_prev) < self.min_pts:
            self.n_reject += 1
            return None
        if X3_pre is not None:
            X3 = np.asarray(X3_pre, float).reshape(-1, 3)
            if len(X3) != len(uv_prev) or len(X3) < self.min_pts:
                self.n_reject += 1
                return None
            good = np.ones(len(uv_prev), bool)
        else:
            X3, good = assign_depth(uv_prev, lidar_uv, lidar_z, self.K,
                                    self.depth_rad, self.depth_rel, self.depth_abs)
            if len(X3) < self.min_pts:
                self.n_reject += 1
                return None
        uv = uv_cur[good]
        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            X3.reshape(-1, 1, 3), uv.reshape(-1, 1, 2), self.K, None,
            flags=cv2.SOLVEPNP_EPNP, reprojectionError=self.reproj_err,
            iterationsCount=self.ransac_iters, confidence=self.conf)
        if not ok or inl is None or len(inl) < self.min_inliers:
            self.n_reject += 1
            return None
        idx = inl.reshape(-1)
        R, _ = cv2.Rodrigues(rvec)
        dT = rigid(R, tvec.reshape(3))          # T_{k ← k-1}
        T_between = inv(dT)                     # T_{k-1 ← k}，给 BetweenFactor
        cov = pnp_covariance(X3[idx], uv[idx], T_between, self.K)
        if cov is not None and self.cov_scale is not None:
            kr, kt = self.cov_scale
            cov = cov.copy()
            cov[:3, :3] *= kr * kr
            cov[3:, 3:] *= kt * kt

        self.T = self.T @ T_between             # T_c0←ck = T_c0←c(k-1) · T_{k-1←k}
        self.n_ok += 1
        r = _reproj(T_between, X3[idx], uv[idx], self.K)
        return dict(ok=True, T_between=T_between, cov6=cov,
                    n_inliers=int(len(idx)), n_used=int(len(X3)),
                    rms_px=float(np.sqrt((r * r).mean())),
                    t_norm=float(np.linalg.norm(tvec)))

    def pose(self):
        return self.T.copy()
