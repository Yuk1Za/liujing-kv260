#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PL 视觉前端 ROS2 节点（支持「以雷达 topic 为节拍」的时间戳对齐）

两种驱动模式，由参数 sync_topic 决定：

  1) 定时器模式   sync_topic:=''（默认，向后兼容）
     自走 rate Hz，按文件名顺序回放图像序列，与雷达各自独立。

  2) 雷达节拍模式  sync_topic:='/velodyne_points'
     订阅雷达发布的 PointCloud2，每收到一条消息：
        取 header.stamp（= kitti_pointcloud_publisher 写入的 KITTI 原始时间戳）
        -> 据此定位【同一帧】的相机图
        -> 立刻跑 PL 角点检测 + 稀疏 LK 并发布
     视觉不再自走定时器：帧号与节拍完全由雷达决定。

角点检测器（参数 detector，默认 auto = 有 ST 就用 ST）：

  detector:='st'   **Shi-Tomasi 响应图语义（阶段 5 主用）**
     PL 输出的是 (H,W) uint8 的 λ_min 响应图（值 = λ_min >> XF_RESP_SHIFT，饱和 255，
     阈值以下写 0），不是二值角点图。于是 CPU 侧必须补上 cv2.goodFeaturesToTrack 的
     后处理：**按响应值降序 -> MIN_DIST 欧氏抑制贪心 -> 取前 max_corners**。
     这与 LVI-SAM / VINS 前端 `goodFeaturesToTrack(img, MAX_CNT, MIN_DIST, mask)` 等价，
     也是 ST 相对 FAST 的唯一价值（FAST 只有二值，无法排序、无法按质量取点）。
         * `threshold` = λ_min 原始值（0…1024），不是 cv2 的 qualityLevel 比例。
           cv2 qualityLevel=0.01 在 KITTI 灰度图上约对应 7；工程里常用 20–40。
         * 抑制规则与 OpenCV 完全一致：**欧氏距离** dx²+dy² < min_dist²（严格小于），
           实现上用 cell=min_dist 的网格只查 3×3 邻域（featureselect.cpp）。

  detector:='fast'  旧的二值 FAST 路径，保留做 A/B 回归（二值图只能均匀取样）。

稀疏 LK 的帧语义（★ 2026-10-17 修复，之前是错的）：

  `sparse_lk(img_prev, img_cur, corners, flow_io, status, ...)` 里 **corners 是 img_prev 上的
  坐标**——内核在 `img_prev` 的该位置取 11×11 patch 算 G 与 Ix/Iy，再去 `img_cur` 上找匹配。
  旧版把「当前帧检测到的角点」直接喂进去，等价于"用当前帧的角点在上一帧取 patch"：
  板端实测在转弯帧中位流**符号都反了**（PL +0.44 / cv2 -1.88），这也正是下游
  `pl_vision_branch.py` 需要 `yaw_sign=-1` 兜符号的根源。

  现在按 VINS 的语义走：**在第 k-1 帧检测 → 跟踪到第 k 帧**，所以角点检测比光流慢一帧。

  `flow_io` 是**迭代初值**（in/out）。旧版从不初始化，首帧读到的是 allocate 出来的垃圾；
  而且冷启 0 会让中位流量系统性压向 0——板端实测同一帧冷启 0.86 px / 用真值当初值 3.88 px
  （cv2 3.87），是典型的局部极小。于是加了 `lk_prior`：
    'affine'（默认）= 用上一帧流场拟合 u,v ~ a+b·x+c·y，按本帧角点坐标求初值
    'const'         = 上一帧有效点 (u,v) 的中位数，全部点同初值
    'none'          = 全 0（旧行为，留作 A/B）
  急转弯段（帧 295–306）平均 |Δ中位流|：none 3.36 / const 2.60 / affine **2.10** px。

帧定位（参数 align）：
  align:=stamp（默认）  用雷达 stamp 在 image_02/timestamps.txt 中做最近邻查找。
                        KITTI _sync 两路同帧率（~9.6 Hz，帧间隔 ~104 ms），
                        同序号帧时间差仅 ~1.5 ms，最近邻稳定命中同序号帧。
  align:=index          直接按雷达消息序号取同序号图像（不查表，最省事）。

发布：
  pl_vision/keypoints  Float32MultiArray  [x0,y0,x1,y1,...]              (N x 2)
      = **本帧观测到的角点**（上一帧的角点 + 稀疏 LK 光流）。首帧无 LK 时退化为本帧检测点。
  pl_vision/flow       Float32MultiArray  [x,y,u,v,status,frame, ...]    (N x 6, 末列=帧号)
      * (x,y) 是**上一帧**的坐标（= 送给 IP 的 corners，内核就是在 img_prev 的该位置取 patch）
      * (u,v) 是 prev -> cur 的位移；本帧观测位置 = (x+u, y+v)
      * status=0 表示该点跟踪失败，此时 (u,v) 是 flow 缓冲区里的**残留值**，必须丢弃
  pl_vision/timing     Float32MultiArray  [frame,kitti_stamp,det_ms,lk_us,total_ms,N,
                                           n_cand,n_blocked]        (前 6 列布局未变)
      * det_ms = 检测器整段耗时（搬图 + IP + 回读），ST/FAST 两种检测器同口径
      * N      = 本帧**跟踪成功**的点数（以前是"检测点数"，口径已变，见 9.9）
      * n_cand = 过阈值且在边界安全区内的候选像素数；n_blocked = 被 MIN_DIST 抑制掉的数量
  pl_vision/sync       Float64MultiArray  [seq,radar_idx,img_idx,radar_stamp,img_stamp,
                                           skew_ms,recv_wall,proc_ms,N,status]
      * sync 必须用 float64：epoch 秒需 31 bit，再乘 1e6 需 51 bit，
        而 float32 尾数只有 24 bit（1.3e9 量级上的 ULP≈128 s）-> 会丢成垃圾。
      * status: 0=已处理 1=重复或乱序跳过 2=越界 3=读图失败

用法（板端）：
  # 定时器模式（旧行为）
  sudo bash run_node.sh --ros-args -p rate:=1.5
  # 雷达节拍模式
  sudo bash run_node.sh --ros-args -p sync_topic:=/velodyne_points -p align:=stamp
"""
import os
import glob
import time
import shutil
from datetime import datetime

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2

from pynq import Overlay, allocate

# ---------------- PL 寄存器（与板端已验证脚本一致） ----------------
# FAST（旧路径，二值角点图）
F_AP, F_IN, F_TH, F_OUT, F_R, F_C = 0x00, 0x10, 0x1c, 0x24, 0x30, 0x38
# Shi-Tomasi（阶段 5 合并位里的 shitomasi_accel_0；由板端 ip_dict 实测偏移抄下来，勿猜）
#   img_in/img_out 各占 8 字节（_1=低 32 位 @+0x00，_2=高 32 位 @+0x04）
T_AP, T_IN, T_OUT = 0x00, 0x10, 0x1C
T_ROWS, T_COLS, T_TH, T_K = 0x28, 0x30, 0x38, 0x40
# 稀疏 LK
S_AP, S_PREV, S_CUR, S_COR, S_FLOW, S_ST = 0x00, 0x10, 0x1c, 0x28, 0x34, 0x40
S_N, S_ROWS, S_COLS, S_ITER = 0x4c, 0x54, 0x5c, 0x64

W, H = 640, 480          # IP 固定分辨率
WH, SEARCH = 5, 4
PMAX = WH + SEARCH + 1   # 边界余量 = 10
Q = 6                    # LK 位移定点格式 Q6
MAXIT = 20
POLL = 0.0002            # 轮询间隔（带 sleep，避免忙等抢 PS 控制通路）


def wr64(mm, addr, val):
    mm.write(addr, val & 0xFFFFFFFF)
    mm.write(addr + 4, (val >> 32) & 0xFFFFFFFF)


def pack(img):
    """(H,W) uint8 -> (H,W/4) uint32（IP 按 32 位字读图）"""
    r, c = img.shape
    return img.reshape(r, c // 4, 4).copy().view(np.uint32).reshape(r, c // 4)


def _nearest(arr, v):
    """升序 float64 数组 arr 中与 v 最接近的下标（二分查找，O(log n)）"""
    if arr is None or len(arr) == 0:
        return -1
    j = int(np.searchsorted(arr, v))
    if j <= 0:
        return 0
    if j >= len(arr):
        return len(arr) - 1
    return j if (arr[j] - v) < (v - arr[j - 1]) else j - 1


def _load_ts_file(path):
    """读 KITTI timestamps.txt -> epoch 秒列表。

    KITTI 时间戳是 9 位纳秒，而 %f 只吃到 6 位，必须先截断，
    否则 strptime 抛 'unconverted data remains'，全变 nan。
    """
    out = []
    if not path or not os.path.isfile(path):
        return out
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                s = ln
                if '.' in s:
                    h, frac = s.split('.', 1)
                    s = h + '.' + frac[:6]
                out.append(datetime.strptime(s, '%Y-%m-%d %H:%M:%S.%f').timestamp())
            except Exception:
                out.append(float('nan'))
    return out


def select_min_dist(resp, max_corners, min_dist, max_cand=0):
    """响应图 -> 前 max_corners 个点。逐条复刻 cv2.goodFeaturesToTrack 的 minDistance 规则。

    规则（合成点阵实测确认）：**欧氏距离** dx²+dy² < min_dist²（严格小于）。
    `cell = round(min_dist)` 网格 + 3×3 邻域只是 cv2 的实现技巧；等价规则就是欧氏。
    ★ 它**不是**方形窗口：按「轴对齐方形」实现时，在切比雪夫 19 / 欧氏 26.9 的一对点上
      就会多抑制一个点，同响应同阈值下与 cv2 的 F1 只有 0.79–0.88。

    性能（KV260 A53 实测，这函数唯一的技术难点，反直觉的点都记在这）：
      · `np.nonzero(r)` 在 480×640 上要 **6.4 ms**；`cv2.findNonZero` 只要 **1.24 ms**。
      · numpy 标量读 `m[y,x]` ≈ 5 µs、bytearray 行切片写 ≈ 6 µs、dict.get ≈ 1.5 µs，
        而纯 Python 空循环迭代只要 1 µs ⇒ 逐点掩码类实现（bytearray 圆盘 27 ms、
        扁平 cell 列表 21 ms、dict 网格 23 ms、cv2.rectangle 11 ms）全部劣于
        **向量化贪心**：每轮「取第一个未屏蔽候选 + 一次整数组算抑制集」，
        1451 候选 **13 ms**、671 候选 **6 ms**（与逐点实现逐点一致，已对拍）。
      · 微优化（预分配 out=、int16 差值、每轮紧凑化）实测**无收益**（±5%），别再试。
      · 想更快只有两条路：把 NMS 搬上 PL，或换 C 扩展；`max_cand` 只能靠牺牲质量换速度。

    resp 必须是 uint8（`cv2.findNonZero` 的输入类型）。调用方负责先把边界安全区
    （LK 的 ±PMAX）抹成 0。

    返回 (pts[N,2] int32 (x,y), n_cand, n_drop)
        n_drop = 未入选的候选数（被 MIN_DIST 抑制，或排名靠后没轮到）
    """
    pos = cv2.findNonZero(resp) if max_corners > 0 else None
    if pos is None:
        return np.zeros((0, 2), np.int32), 0, 0
    pos = pos.reshape(-1, 2)
    n_all = len(pos)
    order = np.argsort(-resp[pos[:, 1], pos[:, 0]].astype(np.int32), kind='stable')
    if max_cand > 0 and n_all > max_cand:
        order = order[:max_cand]
    n_cand = len(order)
    n_take = min(int(max_corners), n_cand)
    if n_take <= 0:
        return np.zeros((0, 2), np.int32), n_cand, 0

    P = pos[order]
    if min_dist <= 0:
        return P[:n_take].astype(np.int32), n_cand, n_cand - n_take

    PX = P[:, 0].astype(np.int32)
    PY = P[:, 1].astype(np.int32)
    r2 = int(min_dist) * int(min_dist)
    blocked = np.zeros(n_cand, bool)
    kept = []
    for _ in range(n_take):
        i = int(np.argmin(blocked))
        if blocked[i]:
            break
        kept.append(i)
        dx = PX - PX[i]
        dy = PY - PY[i]
        blocked |= (dx * dx + dy * dy) < r2
    if not kept:
        return np.zeros((0, 2), np.int32), n_cand, n_cand
    return P[kept].astype(np.int32), n_cand, n_cand - len(kept)


def _fit_plane(x, y, z):
    """最小二乘拟合 z ~ a + b·x + c·y（正规方程 3×3）。退化时返回 None。"""
    n = float(x.size)
    if n < 8:
        return None
    sx, sy = float(x.sum()), float(y.sum())
    A = np.array([[n, sx, sy],
                  [sx, float((x * x).sum()), float((x * y).sum())],
                  [sy, float((x * y).sum()), float((y * y).sum())]], np.float64)
    b = np.array([float(z.sum()), float((z * x).sum()), float((z * y).sum())], np.float64)
    try:
        c = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return None
    return c if np.all(np.isfinite(c)) else None


def flow_prior(pts, fpts, fu, fv, mode, lim=6.0):
    """按上一帧流场给本帧的 LK 迭代初值。返回 (len(pts),2) float32，或 None 表示全 0。

    pts  = 本帧要跟踪的角点（上一帧坐标）
    fpts = 上一帧拟合所用的点（上一帧检测点），fu/fv = 那些点的光流（失败为 nan）
    旋转主导时流场在图像上是近线性的，所以仿射模型比常量准；`lim` 防止先验把迭代带飞
    （IP 可测位移本身只有 ±8 px，先验超过 6 px 一定是上一帧出了问题）。
    """
    if mode == 'none' or fpts is None or fu is None or len(fpts) == 0:
        return None
    ok = np.isfinite(fu) & np.isfinite(fv)
    if int(ok.sum()) < 8:
        return None
    px = fpts[ok, 0].astype(np.float64)
    py = fpts[ok, 1].astype(np.float64)
    if mode == 'const':
        mu = float(np.median(fu[ok]))
        mv = float(np.median(fv[ok]))
        return np.tile(np.array([mu, mv], np.float32), (len(pts), 1))
    cu = _fit_plane(px, py, fu[ok].astype(np.float64))
    cvv = _fit_plane(px, py, fv[ok].astype(np.float64))
    if cu is None or cvv is None:
        return None
    x = pts[:, 0].astype(np.float64)
    y = pts[:, 1].astype(np.float64)
    iu = cu[0] + cu[1] * x + cu[2] * y
    iv = cvv[0] + cvv[1] * x + cvv[2] * y
    if not (np.all(np.isfinite(iu)) and np.all(np.isfinite(iv))):
        return None
    return np.stack([np.clip(iu, -lim, lim), np.clip(iv, -lim, lim)], 1).astype(np.float32)


def _ensure_xclbin(bit_path, log):
    """PYNQ 3.0 的 Overlay() 会去找与 .bit 同名的 .xclbin。

    本板 `xclbinutil` 会 SIGSEGV、但包装脚本仍返回 0 -> 生成空 xclbin -> Overlay() 抛
    `FileNotFoundError: .../xxx.xclbin`（错误信息误导，跟路径无关）。这里在加载前把占位
    xclbin 补上，省掉每次手工 `cp`（这个坑已踩 4 次）。
    """
    xc = os.path.splitext(bit_path)[0] + '.xclbin'
    if os.path.exists(xc):
        return xc
    for cand in ('/home/ubuntu/st/shitomasi_accel_r2.xclbin',
                 '/home/ubuntu/merge_st/kv260_merged_st.xclbin',
                 '/home/ubuntu/merge/merged.xclbin'):
        if os.path.exists(cand):
            try:
                shutil.copyfile(cand, xc)
                log('已补占位 xclbin: %s <- %s' % (xc, os.path.basename(cand)))
                return xc
            except Exception as e:
                log('补 xclbin 失败(%s): %s' % (e, xc))
    log('警告：%s 不存在、也找不到可复制的占位 xclbin，Overlay() 很可能报 FileNotFoundError'
        % xc)
    return None


class PlVisionNode(Node):
    def __init__(self):
        super().__init__('pl_vision')
        P = self.declare_parameter
        P('bit', '/home/ubuntu/merge/merged.bit')
        P('image_dir', '/home/ubuntu/kitti_cam/image_02/data')
        P('timestamps', '/home/ubuntu/kitti_cam/image_02/timestamps.txt')
        P('rate', 1.5)
        P('detector', 'auto')     # auto|st|fast：auto = PL 上有 shitomasi_accel_0 就用 ST
        P('threshold', 20)        # st: λ_min 原始值(0…1024)；fast: 二值阈值
        P('max_corners', 100)
        P('min_dist', 20)         # ST 专用：MIN_DIST 最小间距(px)，同 cv2 goodFeaturesToTrack
        P('max_cand', 0)          # ST 专用：参与 MIN_DIST 抑制的候选上限（0=不限；限了省时间但掉点）
        P('st_k', 0)              # ST 专用：k=0 → 纯 Shi-Tomasi min(λ1,λ2)；>0 → Harris 混合
        P('lk_prior', 'affine')   # LK 迭代初值：affine|const|none（见文件头"稀疏 LK 的帧语义"）
        P('keep_ratio', True)     # True = 等比缩放 + 上下补边（避免宽银幕图被压扁）
        P('download', False)      # False = attach 已加载的 PL
        # ---------------- 时间戳对齐 ----------------
        P('sync_topic', '')       # '' = 定时器模式；'/velodyne_points' = 以雷达节拍驱动
        P('align', 'stamp')       # 'stamp' = 按时间戳最近邻取同帧；'index' = 按消息序号取同序号
        P('velo_timestamps', '')  # 可选：雷达 timestamps.txt，用于报告雷达帧号并交叉核对
        P('skew_warn_ms', 20.0)   # 同帧时间差超过它就告警 (ms)
        P('idle_exit_s', 30.0)    # 雷达节拍模式：节拍静默这么久就自行退出
        P('sync_log_every', 50)   # 雷达节拍模式：每 N 条节拍打一条日志（首尾对齐留痕）
        G = lambda n: self.get_parameter(n).value

        self.rate = float(G('rate'))
        self.nc = int(G('max_corners'))
        self.min_dist = max(0, int(G('min_dist')))
        self.max_cand = max(0, int(G('max_cand')))
        self.st_k = int(G('st_k'))
        self.lk_prior = str(G('lk_prior') or 'affine').strip().lower()
        if self.lk_prior not in ('affine', 'const', 'none'):
            raise RuntimeError("lk_prior 只能是 affine/const/none，收到 '%s'" % self.lk_prior)
        self.thr = int(G('threshold'))
        self.keep_ratio = bool(G('keep_ratio'))
        self.sync_topic = str(G('sync_topic') or '').strip()
        self.align = str(G('align') or 'stamp').strip().lower()
        self.skew_warn = float(G('skew_warn_ms'))
        self.idle_exit = float(G('idle_exit_s'))
        self.sync_log_every = max(1, int(G('sync_log_every')))

        # ---- 图像列表 ----
        self.files = sorted(glob.glob(os.path.join(G('image_dir'), '*.png')))
        if not self.files:
            raise RuntimeError('图像目录为空: %s' % G('image_dir'))
        self.ts = _load_ts_file(G('timestamps'))
        self.ts_arr = np.asarray(self.ts, dtype=np.float64)
        self.velo_arr = None
        vt = _load_ts_file(G('velo_timestamps'))
        if vt:
            self.velo_arr = np.asarray(vt, dtype=np.float64)
        self.get_logger().info('图像 %d 帧；相机戳 %d 行；雷达戳 %d 行'
                               % (len(self.files), len(self.ts),
                                 0 if self.velo_arr is None else len(self.velo_arr)))

        # ---- attach PL ----
        t0 = time.perf_counter()
        _ensure_xclbin(str(G('bit')), self.get_logger().warn)
        self.ol = Overlay(G('bit'), download=bool(G('download')))
        self.get_logger().info('Overlay 完成 %.3f s (download=%s)'
                               % (time.perf_counter() - t0, G('download')))
        ips = sorted(self.ol.ip_dict)
        self.get_logger().info('PL IP: %s' % ips)
        if 'slk_accel_0' not in ips:
            raise RuntimeError('PL 上没有 slk_accel_0（确认合并 bit 已加载 / 清过 PYNQ 元数据缓存）')

        # ---- 选角点检测器：ST（响应图）优先，退回 FAST（二值图）----
        want = str(G('detector') or 'auto').strip().lower()
        has_st, has_fast = 'shitomasi_accel_0' in ips, 'fast_accel_0' in ips
        if want == 'auto':
            self.detector = 'st' if has_st else ('fast' if has_fast else '')
        elif want in ('st', 'fast'):
            self.detector = want if (has_st if want == 'st' else has_fast) else ''
        else:
            raise RuntimeError("detector 只能是 auto/st/fast，收到 '%s'" % want)
        if not self.detector:
            raise RuntimeError('PL 上没有可用的角点检测 IP（需要 shitomasi_accel_0 或 fast_accel_0）；'
                               '当前 IP: %s' % ips)
        self.get_logger().info('检测器 = %s（threshold=%d min_dist=%d max_corners=%d，LK 初值=%s）'
                               % (self.detector.upper(), self.thr, self.min_dist, self.nc, self.lk_prior))
        if self.detector == 'st':
            self.tm = self.ol.shitomasi_accel_0.mmio
        else:
            self.fm = self.ol.fast_accel_0.mmio
        self.sm = self.ol.slk_accel_0.mmio

        # ---- 一次分配、全程复用（每帧重新 allocate 会极慢）----
        self.inp = allocate(shape=(H, W), dtype=np.uint8, cacheable=1)
        self.out = allocate(shape=(H, W), dtype=np.uint8, cacheable=1)
        self.pb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
        self.cb = allocate(shape=(H, W // 4), dtype=np.uint32, cacheable=1)
        self.cor_b = allocate(shape=(self.nc,), dtype=np.uint32, cacheable=1)
        self.fl_b = allocate(shape=(self.nc,), dtype=np.uint32, cacheable=1)
        self.st_b = allocate(shape=(self.nc,), dtype=np.uint32, cacheable=1)

        if self.detector == 'st':
            # Shi-Tomasi 固定配置：in/out 各自是独立的 64 位指针寄存器
            wr64(self.tm, T_IN, self.inp.device_address)
            wr64(self.tm, T_OUT, self.out.device_address)
            self.tm.write(T_ROWS, H)
            self.tm.write(T_COLS, W)
            self.tm.write(T_TH, self.thr)
            self.tm.write(T_K, self.st_k)
        else:
            # FAST 固定配置
            wr64(self.fm, F_IN, self.inp.device_address)
            wr64(self.fm, F_OUT, self.out.device_address)
            self.fm.write(F_TH, self.thr)
            self.fm.write(F_R, H)
            self.fm.write(F_C, W)
        # LK 固定配置
        wr64(self.sm, S_PREV, self.pb.device_address)
        wr64(self.sm, S_CUR, self.cb.device_address)
        wr64(self.sm, S_COR, self.cor_b.device_address)
        wr64(self.sm, S_FLOW, self.fl_b.device_address)
        wr64(self.sm, S_ST, self.st_b.device_address)
        self.sm.write(S_ROWS, H)
        self.sm.write(S_COLS, W)
        self.sm.write(S_ITER, MAXIT)

        # ---- publishers ----
        self.pub_kp = self.create_publisher(Float32MultiArray, 'pl_vision/keypoints', 10)
        self.pub_flow = self.create_publisher(Float32MultiArray, 'pl_vision/flow', 10)
        self.pub_t = self.create_publisher(Float32MultiArray, 'pl_vision/timing', 10)
        self.pub_sync = self.create_publisher(Float64MultiArray, 'pl_vision/sync', 10)

        # ---- 状态 ----
        self.i = 0             # 定时器模式：下一个要处理的图序号
        self.last_idx = -1     # 已处理过的最大图序号（去重 / 跳帧判据）
        self.prev = None       # 上一帧灰度图（LK 的 img_prev）
        self.track_pts = None  # 上一帧**检测**到的角点（= 下一帧要跟踪的 corners，属 prev 帧坐标）
        self.field_pts = None  # 上一次跟踪所用的点 + 结果，仅用于拟合下一帧的流场先验
        self.field_u = None
        self.field_v = None
        self.lk_dt = 0.0       # 最近一次 LK 内核耗时（秒）
        self.n_msg = 0         # 收到雷达消息数（雷达节拍模式）
        self.last_msg_wall = None

        if self.sync_topic:
            self.sub = self.create_subscription(PointCloud2, self.sync_topic, self.on_radar, 10)
            self.wd = self.create_timer(1.0, self.watchdog)
            self.get_logger().info(
                '就绪【雷达节拍模式】：topic=%s align=%s max_corners=%d idle_exit=%.0f s'
                % (self.sync_topic, self.align, self.nc, self.idle_exit))
        else:
            self.timer = self.create_timer(1.0 / self.rate, self.tick)
            self.get_logger().info('就绪【定时器模式】：rate=%.2f Hz  threshold=%d  max_corners=%d'
                                   % (self.rate, int(G('threshold')), self.nc))

    # ---------------------- 工具 ----------------------
    def _grab(self, path):
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            return None
        h0, w0 = img.shape[:2]
        if (w0, h0) != (W, H):
            if self.keep_ratio:
                # 等比缩放到宽 W，再上下居中补零到 H。
                # KITTI image_02 是 1242x375（宽银幕），直接拉伸到 640x480 会横向压扁 ~2.5 倍，
                # 纹理被破坏 -> LK 迭代变多、精度下降。等比 + 黑边更符合真实几何。
                h2 = min(H, max(1, int(round(h0 * W / float(w0)))))
                small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
                canvas = np.zeros((H, W, img.shape[2]), img.dtype)
                top = (H - h2) // 2
                canvas[top:top + h2] = small
                img = canvas
            else:
                img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def _wait(self, mm, reg, what):
        """轮询 AP_DONE（bit1），带超时保护"""
        t0 = time.perf_counter()
        while not ((mm.read(reg) >> 1) & 1):
            if time.perf_counter() - t0 > 5.0:
                raise RuntimeError('%s 超时 (>5 s)' % what)
            time.sleep(POLL)
        return time.perf_counter() - t0

    def _pub(self, pub, arr, labels):
        msg = Float32MultiArray()
        msg.data = np.asarray(arr, np.float32).reshape(-1).tolist()
        dim = []
        for k, lab in enumerate(labels):
            d = MultiArrayDimension()
            d.label = lab
            d.size = int(arr.shape[k]) if arr.ndim > k else 0
            d.stride = int(np.prod(arr.shape[k:])) if arr.ndim > k else 0
            dim.append(d)
        msg.layout.dim = dim
        msg.layout.data_offset = 0
        pub.publish(msg)

    def _pub_sync(self, seq, radar_idx, img_idx, rs, is_, skew_ms,
                  recv_wall, proc_ms, n, status):
        """发布配对信息（float64，绝对时间戳不丢精度）"""
        msg = Float64MultiArray()
        msg.data = [float(seq), float(radar_idx), float(img_idx), float(rs), float(is_),
                    float(skew_ms), float(recv_wall), float(proc_ms), float(n), float(status)]
        d = MultiArrayDimension()
        d.label = 'sync'
        d.size = 10
        d.stride = 10
        msg.layout.dim = [d]
        msg.layout.data_offset = 0
        self.pub_sync.publish(msg)

    # ---------------------- 角点检测（PL） ----------------------
    def _detect(self, cur):
        """返回 (pts[N,2] int32(x,y), det_ms, n_cand, n_blocked)"""
        return self._detect_st(cur) if self.detector == 'st' else self._detect_fast(cur)

    def _detect_st(self, cur):
        """Shi-Tomasi：PL 出 λ_min 响应图 -> CPU 补 cv2.goodFeaturesToTrack 的后处理。

        PL 输出 (H,W) uint8，值 = λ_min >> XF_RESP_SHIFT（饱和 255），阈值以下写 0。
        后处理三步（与 OpenCV 完全同序）：
          1) 响应降序排序；
          2) MIN_DIST 欧氏抑制贪心（dx²+dy² < min_dist²，与 cv2 同规则，不是方形窗口）；
          3) 取前 max_corners。
        """
        t0 = time.perf_counter()
        self.inp[:] = cur
        self.inp.sync_to_device()
        # 不需要预清零：板端 15 档 × 340 帧次已验证 IP 每像素都写（0xAA 残留 = 0）
        self.tm.write(T_AP, 1)
        self._wait(self.tm, T_AP, 'Shi-Tomasi')
        self.out.sync_from_device()
        resp = np.array(self.out)          # 必须复制：DMA 缓冲下一帧会被覆盖
        t_det = (time.perf_counter() - t0) * 1e3

        # 边界安全区：LK 搜索需 ±PMAX 余量，越界点直接抹掉（省去后面的布尔索引）
        resp[:PMAX, :] = 0
        resp[H - PMAX:, :] = 0
        resp[:, :PMAX] = 0
        resp[:, W - PMAX:] = 0

        pts, n_cand, n_drop = select_min_dist(resp, self.nc, self.min_dist, self.max_cand)
        return pts, t_det, n_cand, n_drop

    def _detect_fast(self, cur):
        """旧的 FAST 二值路径：只能「沿序列均匀间隔取样」（保留做 A/B 回归）"""
        t0 = time.perf_counter()
        self.inp[:] = cur
        self.inp.sync_to_device()
        self.out.fill(0)                   # FAST 只写角点，非角点位置保持预填值
        self.out.sync_to_device()
        self.fm.write(F_AP, 1)
        self._wait(self.fm, F_AP, 'FAST')
        self.out.sync_from_device()
        ys, xs = np.nonzero(np.array(self.out))
        t_det = (time.perf_counter() - t0) * 1e3
        n_all = int(xs.size)
        if n_all:
            m = (xs >= PMAX) & (ys >= PMAX) & (xs < W - PMAX) & (ys < H - PMAX)
            sel = np.stack([xs[m], ys[m]], 1).astype(np.int32)
            # ★ 必须「沿序列均匀间隔取样」，不能取前 nc 个：
            #   np.nonzero 是行优先顺序，取前 N 个会让角点全挤在图像上部，
            #   下游按光流统计运动时会严重有偏（pl_vs_cv2.py 里早已改成均匀取样，节点漏改了）。
            if len(sel) > self.nc:
                sel = sel[np.linspace(0, len(sel) - 1, self.nc).astype(np.int64)]
            pts = sel
        else:
            pts = np.zeros((0, 2), np.int32)
        return pts, t_det, n_all, 0

    # ---------------------- 单帧处理（两种模式共用） ----------------------
    def _lk_track(self, prev, cur, corners, prior):
        """把 prev 帧的角点跟踪到 cur。**corners 必须是 prev 帧坐标**（内核在 prev 上取 patch）。

        返回 (u, v, status) 三个长度 N 的数组；status==1 时 (u,v) 有效。
        """
        n = len(corners)
        self.pb[:] = pack(prev)
        self.pb.sync_to_device()
        self.cb[:] = pack(cur)
        self.cb.sync_to_device()
        cor = (corners[:, 1].astype(np.uint32) << 16) | (corners[:, 0].astype(np.uint32) & 0xFFFF)
        self.cor_b[:n] = cor
        if prior is None:
            self.fl_b[:n] = 0          # ★ 必须显式清零：内核把 flow_io 当迭代初值
        else:
            fu = np.clip(np.round(prior[:, 0] * (1 << Q)), -32767, 32767).astype(np.int64)
            fv = np.clip(np.round(prior[:, 1] * (1 << Q)), -32767, 32767).astype(np.int64)
            self.fl_b[:n] = (((fu & 0xFFFF).astype(np.uint32)) << 16) | (fv & 0xFFFF).astype(np.uint32)
        self.fl_b.sync_to_device()
        self.cor_b.sync_to_device()
        self.sm.write(S_N, n)
        self.sm.write(S_AP, 1)
        self.lk_dt = self._wait(self.sm, S_AP, 'LK')
        self.fl_b.sync_from_device()
        self.st_b.sync_from_device()
        raw = np.array(self.fl_b).view(np.int16).reshape(self.nc, 2)[:n]
        u = raw[:, 1].astype(np.float32) / (1 << Q)
        v = raw[:, 0].astype(np.float32) / (1 << Q)
        return u, v, np.array(self.st_b)[:n].astype(np.uint8)

    def _process(self, idx):
        """处理第 idx 帧图：先把上一帧角点跟踪到本帧，再在本帧检测（供下一帧跟踪）。

        注意顺序：**检测比光流慢一帧**——本帧发出去的光流用的是【上一帧】检测的角点，
        这是 sparse_lk 内核的语义要求（corners 落在 img_prev 上），也是 VINS 的做法。

        返回 (N, det_ms, lk_us, total_ms) 或 None；N = 本帧跟踪成功的点数。
        """
        t_all0 = time.perf_counter()
        cur = self._grab(self.files[idx])
        if cur is None:
            self.get_logger().warn('读图失败: %s' % self.files[idx])
            return None

        # ---- ① 稀疏 LK：上一帧角点 -> 本帧 ----
        t_lk_us = 0.0
        u = v = None
        st = np.zeros(0, np.uint8)
        tracked = np.zeros((0, 2), np.int32)
        if self.prev is not None and self.track_pts is not None and len(self.track_pts):
            prior = flow_prior(self.track_pts, self.field_pts, self.field_u,
                               self.field_v, self.lk_prior)
            u, v, st = self._lk_track(self.prev, cur, self.track_pts, prior)
            good = st == 1
            n_ok = int(good.sum())
            if n_ok:
                t_lk_us = self.lk_dt * 1e6 / n_ok
                tracked = self.track_pts[good] + np.stack([u[good], v[good]], 1).round().astype(np.int32)
            # 记下这次跟踪所用的点与结果，供下一帧拟合流场先验
            self.field_pts, self.field_u, self.field_v = self.track_pts, u, v

        # ---- ② 本帧检测（PL：ST 响应图 或 FAST 二值图）：给下一帧当 corners ----
        pts, t_det, n_cand, n_blocked = self._detect(cur)
        N = len(tracked)

        t_all = (time.perf_counter() - t_all0) * 1e3

        # ---- 发布 ----
        kp = tracked.astype(np.float32) if N else np.zeros((0, 2), np.float32)
        self._pub(self.pub_kp, kp, ('N', 'xy'))
        # flow: [x_prev, y_prev, u, v, status, frame]，x,y 是送进 IP 的 corners（上一帧坐标）
        nf = 0 if u is None else len(u)
        flow6 = np.zeros((nf, 6), np.float32)
        if nf:
            flow6[:, 0] = self.track_pts[:, 0]
            flow6[:, 1] = self.track_pts[:, 1]
            flow6[:, 2] = u
            flow6[:, 3] = v
            flow6[:, 4] = st
            flow6[:, 5] = idx        # 下游据此把光流与配对台账对上，不必依赖到达顺序
        self._pub(self.pub_flow, flow6, ('N', 'xyuvfs'))
        stamp = self.ts[idx] if idx < len(self.ts) else float('nan')
        self._pub(self.pub_t, np.array([[idx, stamp, t_det, t_lk_us, t_all, N,
                                         n_cand, n_blocked]], np.float32),
                  ('row', 'vals'))

        if idx % 20 == 0 or idx < 3:
            self.get_logger().info(
                'frame %d/%d | %s %.2f ms (候选 %d/%d 检测点) | LK %d/%d 成功 %.2f us/点 | 总 %.2f ms%s'
                % (idx, len(self.files), self.detector.upper(), t_det, n_cand, len(pts), N, nf,
                   t_lk_us, t_all,
                   '' if nf else '  [首帧无 LK]'))

        self.prev = cur
        self.track_pts = pts
        return N, t_det, t_lk_us, t_all

    # ---------------------- 模式 1：定时器 ----------------------
    def tick(self):
        if self.i >= len(self.files):
            self.get_logger().info('回放结束，共处理 %d 帧' % self.i)
            self.timer.cancel()
            return
        if self._process(self.i) is not None:
            self.last_idx = self.i
        self.i += 1

    # ---------------------- 模式 2：雷达节拍 ----------------------
    def on_radar(self, msg):
        t_recv = time.time()
        self.n_msg += 1
        self.last_msg_wall = t_recv
        seq = self.n_msg
        rs = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        # 雷达侧帧号：有雷达 timestamps.txt 就用最近邻（对丢消息鲁棒），否则用消息计数
        radar_idx = _nearest(self.velo_arr, rs) if self.velo_arr is not None else (seq - 1)
        # 相机侧帧号：同帧索引
        if self.align == 'index':
            img_idx = radar_idx
        else:
            img_idx = _nearest(self.ts_arr, rs)

        img_stamp = self.ts[img_idx] if 0 <= img_idx < len(self.ts) else float('nan')
        skew_ms = (img_stamp - rs) * 1e3 if img_stamp == img_stamp else float('nan')

        if self.align == 'stamp' and self.velo_arr is not None and img_idx >= 0:
            k = _nearest(self.velo_arr, rs)
            if abs(k - img_idx) > 1:
                self.get_logger().warn('帧号不一致：雷达#%d vs 相机#%d（stamp 最近邻）' % (k, img_idx))

        if (seq <= 3 or seq % self.sync_log_every == 0
                or (skew_ms == skew_ms and abs(skew_ms) > self.skew_warn)):
            self.get_logger().info('节拍 seq=%d 雷达#%d 相机#%d skew=%.2f ms %s'
                                   % (seq, radar_idx, img_idx, skew_ms,
                                      '' if self.prev is not None else '[首帧无 LK]'))

        if img_idx < 0 or img_idx >= len(self.files):
            self._pub_sync(seq, radar_idx, img_idx, rs, img_stamp, skew_ms, t_recv, 0.0, 0, 2)
            return
        if img_idx <= self.last_idx:
            # 同一帧被两条雷达消息命中 / 乱序 -> 不重复计算
            self._pub_sync(seq, radar_idx, img_idx, rs, img_stamp, skew_ms, t_recv, 0.0, 0, 1)
            return

        gap = img_idx - self.last_idx
        if self.last_idx >= 0 and gap != 1:
            self.get_logger().warn('帧号跳跃 %d -> %d（漏 %d 帧），本帧 LK 位移将跨越 %d 个帧间隔'
                                   % (self.last_idx, img_idx, gap - 1, gap))

        r = self._process(img_idx)
        if r is None:
            self._pub_sync(seq, radar_idx, img_idx, rs, img_stamp, skew_ms, t_recv, 0.0, 0, 3)
            self.last_idx = img_idx
            return
        N, _tf, _tl, ta = r
        self.last_idx = img_idx
        self._pub_sync(seq, radar_idx, img_idx, rs, img_stamp, skew_ms, t_recv, ta, N, 0)

    def watchdog(self):
        """雷达节拍模式：节拍停了就自己退出（否则节点会挂着不结束）"""
        if self.last_msg_wall is None:
            return
        if time.time() - self.last_msg_wall > self.idle_exit:
            self.get_logger().info('雷达节拍静默 > %.0f s，视觉分支退出（已处理 %d 帧，收 %d 条消息）'
                                   % (self.idle_exit, self.last_idx + 1, self.n_msg))
            self.wd.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PlVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
