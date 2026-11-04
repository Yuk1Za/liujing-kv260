#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vo_pnp_node —— 视觉 PnP 里程计 ROS2 节点（上板形态）。

订阅 /velodyne_points，以**点云为节拍**每 `every` 帧唤醒一次：
  上一帧图像 + 当前帧图像 做匹配 → 上一帧点云投影给特征赋深 → solvePnPRansac → 相对位姿
发布**相对位姿 + 6×6 协方差**（正好是 `map_optimization` 里 BetweenFactor<Pose3> 的输入），
以及累积位姿（便于 RViz 观察）。

匹配前端两套，用 `--frontend cv2|pl` 切换（A/B 用）：
  cv2 : `goodFeaturesToTrack(0.05, 20px) + calcOpticalFlowPyrLK(21×21, 3 层)`  板上 ~43–48 ms
  pl  : `pl_match.PLMatcher`（PL FAST + PL 稀疏 LK 软件金字塔）              板上 ~15 ms

两个必须说清的设计点
--------------------
1. **降频是硬约束，不是优化**：板上实测每帧 ~63–70 ms（投影 17.5 + 前端 8–48 + step 37.5），
   8 Hz 会占 0.5 核，而系统已 405% 饱和。`every=4` ⇒ 2 Hz ⇒ 0.13–0.14 核。
   视觉约束对 ISAM2 位姿图是**冗余**信息（激光每帧都有里程计因子），降频不影响后端质量。
2. **降频不影响精度**：唤醒时用的是 (k−1, k) **相邻**两帧，不是跨 every 帧。
   所以每帧都要缓存图像与点云，唤醒时才计算。
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2, Image, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32MultiArray

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vo_core import PnPVO, rigid

# ★ 不能用 expanduser('~')：PL 前端要 root 才能加载位流，`sudo` 下 $HOME 变成 /root
#   ⇒ 图片路径全错、imread 返回 None、每次唤醒都被判"未配对"（踩过）。
H = os.environ.get('LVI_HOME', '/home/ubuntu')
CAL = f'{H}/kiss_icp_fpga/evidence/reference_raw_v3'
IMG_DIR = f'{H}/kitti_cam/image_02/data'


# 预缩放灰度图目录（make_pre.py 生成）。设了它就走「在线只解小图」的快路径：
# 板上实测 imread(1226×370 彩色) 34.5 + resize 18.5 = 53 ms/张，而直读 640×480 灰度只要 **5.7 ms/张**，
# 每唤醒省 ~94 ms（原 105 ms → 11 ms）。像素由 make_pre.py 逐位校验与原路径相同 ⇒ 精度不变。
IMG_PRE = os.environ.get('IMG_PRE_DIR', '')
DEPTH_DIR = os.environ.get('DEPTH_DIR', '')
DEPTH_SCALE = float(os.environ.get('DEPTH_SCALE', '1000'))


def grab_gray(idx, wimg, himg):
    """按帧号取**灰度**图。有预缩放目录就直读，否则走原路径（BGR 读图 → letterbox → 转灰度）。"""
    if IMG_PRE:
        g = cv2.imread(f'{IMG_PRE}/{idx:010d}.png', cv2.IMREAD_GRAYSCALE)
        if g is not None and g.shape == (himg, wimg):
            return g
    im = grab_img(idx, wimg, himg)
    return None if im is None else cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)


def grab_img(idx, wimg, himg):
    """按帧号从磁盘读图并 letterbox（与 pl_vision_node._grab 同实现）。

    ★ 图像**不走 DDS**：板上实测同一个 ~1 MB 的图像消息 + 960 KB 点云消息，
      通过 DDS 传输把回放节拍拖到 ~2.2 s/帧（本应 0.1 s）。板上既有视觉节点
      也是本地读图（只订阅 /velodyne_points），这里保持同一口径。
    ★ `sudo` 下进程的 cwd/HOME 都可能是 /root，所以 IMG_DIR 用绝对路径常量。
    """
    im = cv2.imread(f'{IMG_DIR}/{idx:010d}.png')
    if im is None:
        return None
    nh = int(round(im.shape[0] * wimg / im.shape[1]))
    r = cv2.resize(im, (wimg, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((himg, wimg, 3), np.uint8)
    top = (himg - nh) // 2
    out[top:top + nh] = r
    return out


_DT = {PointField.INT8: np.int8, PointField.UINT8: np.uint8, PointField.INT16: np.int16,
       PointField.UINT16: np.uint16, PointField.INT32: np.int32, PointField.UINT32: np.uint32,
       PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}


def pc2_xyz(msg):
    """按 fields 的 offset/datatype 解析 PointCloud2 → (N,3) float64（不假定 x,y,z 连续）。"""
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, np.uint8)
    cols = []
    for name in ('x', 'y', 'z'):
        f = next((q for q in msg.fields if q.name == name), None)
        if f is None:
            raise RuntimeError(f'PointCloud2 缺字段 {name}')
        cols.append(np.ndarray((n,), dtype=_DT[f.datatype], buffer=buf,
                               offset=f.offset, strides=(msg.point_step,)))
    return np.stack(cols, 1).astype(np.float64)


def img_msg_to_bgr(msg):
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


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


def proj_matrix():
    """点云(velo) → cam2 的 4×4。与服务器 vo_core_eval / pnp_vo 同一口径。"""
    vc = load_calib(f'{CAL}/calib_velo_to_cam.txt')
    cc = load_calib(f'{CAL}/calib_cam_to_cam.txt')
    P2 = cc['P_rect_02'].reshape(3, 4)
    t_off = np.array([P2[0, 3] / P2[0, 0], P2[1, 3] / P2[1, 1], 0.0])
    T_v_c0 = rigid(cc['R_rect_00'].reshape(3, 3), np.zeros(3)) @ \
             rigid(vc['R'].reshape(3, 3), vc['T'].ravel())
    return rigid(np.eye(3), -t_off) @ T_v_c0


class VoPnp(Node):
    def __init__(self, a):
        super().__init__('vo_pnp')
        self.a = a
        self.K = np.array([[a.fx, 0, a.cx], [0, a.fx, a.cy], [0, 0, 1]], float)
        # ★ --tvc：外参覆盖（M2DGR-plus 等 KITTI 之外的数据集）
        if getattr(a, 'tvc', ''):
            self.T_v_c2 = np.loadtxt(a.tvc).reshape(4, 4)
            print(f'[VO] T_v_c2 由 --tvc 覆盖:\n{self.T_v_c2}')
        else:
            self.T_v_c2 = proj_matrix()
        self.vo = PnPVO(self.K, reproj_err=a.reproj, ransac_iters=a.iters)

        # PL 前端：--frontend pl / hybrid 时构造（需要 root + 位流）
        self.plm = None
        if a.frontend in ('pl', 'hybrid', 'st', 'stcv2'):
            from pl_match import PLMatcher
            self.plm = PLMatcher(bit=a.pl_bit, th=a.pl_th, max_cnt=a.max_cnt,
                                 min_dist=a.min_dist, levels=a.pl_levels,
                                 max_iter=a.pl_iters, w=a.wimg, h=a.himg, q=a.pl_q,
                                 st_th=a.st_th, max_cand=a.max_cand, nms=a.nms)
            self.get_logger().info(
                f'PL 前端就绪：pynq {self.plm.ver}  FAST th={a.pl_th}  '
                f'levels={a.pl_levels} iters={a.pl_iters} Q={a.pl_q}  '
                f'max_cnt={a.max_cnt}  位流 {os.path.basename(a.pl_bit)}')

        # ★ RELIABLE + depth=50：BEST_EFFORT 在整链满载时把 265 帧丢到只剩 75
        #   （DDS 侧直接丢弃，回调根本看不到），配对间隙拉大后 LK 追不上
        #   ⇒ step 连败 ⇒ 间隙更大（死亡螺旋）。主链对这个话题本来就是 RELIABLE
        #   （见 HANDOFF：点云 QoS 必须 RELIABLE），视觉节点跟着走才拿得到消息。
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.sub_pc = self.create_subscription(PointCloud2, a.pc_topic, self.on_pc, qos)
        self.p_rel = self.create_publisher(PoseWithCovarianceStamped, '/vo_pnp/rel', 10)
        self.p_abs = self.create_publisher(PoseStamped, '/vo_pnp/abs', 10)
        self.p_t = self.create_publisher(Float32MultiArray, '/vo_pnp/timing', 10)

        # ★ 必须按 header.stamp 配对：图像与点云是两个独立话题，DDS **不保证到达顺序**
        #   （publisher 先 publish 图也无效，不同话题走不同通道）。靠隐含顺序会让
        #   on_pc 拿到上一帧的图 ⇒ 整条链错位一帧 ⇒ 实测误差 46 m / 旋转 52°。
        self.img_buf = {}
        self.dep_buf = {}        # 帧号 -> 灰度图（本地读取缓存）
        self.n_rgbd = 0
        self.raw_buf = {}        # stamp_ns -> (N,3)（雷达回退时才解出来）
        self.msg_buf = {}        # stamp_ns -> PointCloud2（懒解码源）
        self.prev_pc_ns = None
        # ★★ 上一次**成功处理**的帧号 + 帧号→点云 stamp 的映射。
        #   原实现固定配对 (f_cur-1, f_cur)，但 every>1 时只处理 1/4 的帧
        #   ⇒ 累积位姿只累加了 1/4 的真实运动（实测 |Tvis| 只有应有的 1/4，
        #   进而 dT 高达 1.4 m、dR 12°）。必须按"上次处理帧 → 当前帧"配对。
        self.prev_frame = None
        self.last_recv = None     # 上一个收到的帧号（首唤醒配对用）
        self.frame_ns = {}
        self.n_pc = 0
        self.n_drop = 0          # 因图/点云未配对而丢弃的唤醒
        self.n_wake = 0
        self.n_ok = 0
        self.n_reanchor = 0
        self.n_abs_pub = 0
        self._orb = None        # ORB 前端（懒建）
        self._bf = None
        self.acc = {'gray': 0.0, 'proj': 0.0, 'match': 0.0, 'step': 0.0}
        self.acc_pl = {}         # PL 内部分段累计
        self.dump = open(a.dump, 'w') if a.dump else None
        # ★ 帧时间戳表（每行一个秒，行号=图像帧号）：bag 回放时 frame_id 无帧号用它查
        self.stamp_tbl = None
        if getattr(a, 'stamps', ''):
            try:
                self.stamp_tbl = np.loadtxt(a.stamps)
                self.get_logger().info(f'帧戳表 {a.stamps}：{len(self.stamp_tbl)} 帧')
            except Exception as e:
                self.get_logger().warn(f'帧戳表读取失败（退回接收计数）: {e}')

        self.get_logger().info(
            f'vo_pnp 就绪：out_frame={a.out_frame} 前端 {a.frontend}  每 {a.every} 帧唤醒（{a.rate/a.every:.1f} Hz）  '
            f'K fx={a.fx:.2f} cx={a.cx:.2f} cy={a.cy:.2f}  '
            f'max_cnt={a.max_cnt}  pc={a.pc_topic} 图=本地 {IMG_DIR}')

    # ---------------- 回调 ----------------
    def _img(self, frame_idx):
        if frame_idx not in self.img_buf:
            self.img_buf[frame_idx] = grab_gray(frame_idx, self.a.wimg, self.a.himg)
            if len(self.img_buf) > 6:
                for k in sorted(self.img_buf)[:-4]:
                    self.img_buf.pop(k, None)
        return self.img_buf.get(frame_idx)

    def _depth(self, frame_idx):
        """按帧号取 **对齐深度图**（float32 米）。没有 DEPTH_DIR 或文件缺失返回 None。"""
        if not DEPTH_DIR:
            return None
        if frame_idx not in self.dep_buf:
            p = f'{DEPTH_DIR}/{frame_idx:010d}.png'
            d = cv2.imread(p, cv2.IMREAD_UNCHANGED)
            self.dep_buf[frame_idx] = (None if d is None
                                       else d.astype(np.float32) / DEPTH_SCALE)
            if len(self.dep_buf) > 6:
                for k in sorted(self.dep_buf)[:-4]:
                    self.dep_buf.pop(k, None)
        return self.dep_buf.get(frame_idx)

    def _frame_by_stamp(self, ns):
        """无帧号发布端（M2DGR bag 的 frame_id=/rslidar）：按时间戳查帧戳表定帧号。

        ★ 为什么不能用接收计数（2026-10-27 实测踩坑）：BEST_EFFORT + 节点忙时丢消息，
          计数 0..66 对应真实帧 2..263 ⇒ 图像按 0..66 取（只覆盖前 13 s 近静止段），
          视觉链 52 s 只累计 1.7 m（真值 15 m），因子约束整个"没动"。
        """
        if self.stamp_tbl is None or not len(self.stamp_tbl):
            return self.a.F0 + self.n_pc - 1
        t = ns * 1e-9
        i = int(np.argmin(np.abs(self.stamp_tbl - t)))
        return self.a.F0 + i

    def on_pc(self, msg):
        ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        # ★ RGB-D 赋深路径用不到雷达点云 ⇒ 只存 msg 对象、用时再解码
        #   （57k 点 × 16 B 的解析 ~10 ms/帧，重载下直接吃掉半个消息周期）
        self.msg_buf[ns] = msg
        if len(self.msg_buf) > 12:
            for k in sorted(self.msg_buf)[:-8]:
                self.msg_buf.pop(k, None)
        self.n_pc += 1

        # ★ 帧号来自 frame_id，不靠计数：BEST_EFFORT 丢帧时计数会与真实帧号脱节
        try:
            f_cur = int(msg.header.frame_id.split(':')[1])
        except (IndexError, ValueError):
            f_cur = self._frame_by_stamp(ns)
        self.frame_ns[f_cur] = ns
        for k in sorted(self.frame_ns)[:-24]:
            self.frame_ns.pop(k, None)
        self._depth(f_cur)          # 预取：下次它成为 f_prev 时缓存命中

        if f_cur % self.a.every == 0 and self.prev_pc_ns is not None:
            # ★★ 配对「上一次成功处理的帧 → 当前帧」：every=4 时这一对间隔 4 帧、
            #   位移是单帧的 4 倍，累积出来的 abs 才等于真实运动。
            f_prev = self.prev_frame if self.prev_frame is not None else self.last_recv
            # ★ 间隙保险：步失败时 prev 不动、间隙拉大，超过上限后 LK 必崩
            #   （实测特征数 76→29 一路掉）。此时重锚，宁可留一个小空洞。
            if f_prev is not None and f_cur - f_prev > 8:
                self.get_logger().warn(f'间隙 {f_cur - f_prev} > 8，重锚到 f{f_cur}（本帧不产因子）')
                self.n_reanchor += 1
                # ★ 重锚只能重置状态，本帧必须跳过：若把 f_prev 设成 f_cur 还继续处理，
                #   读的是同一张图 ⇒ uv_prev==uv_cur ⇒ PnP 解出 R=I/t=0 的恒等因子，
                #   而恒等因子会把关键帧相对位姿钉死（bridge1 实测 31 次重锚 ⇒ 每条
                #   都注入 |Tvis|≈1e-8 的垃圾约束，把退化场景的结果搅成随机）。
                self.prev_frame = f_cur
                self.prev_pc_ns = ns
                return
            ns_prev = self.frame_ns.get(f_prev)
            raw_k1 = self.raw_buf.get(ns_prev) if ns_prev is not None else None
            img_k = self._img(f_cur)
            img_k1 = self._img(f_prev) if f_prev is not None else None
            if os.environ.get('VO_DEBUG_PAIR') and self.n_wake <= 3:
                self.get_logger().info(f'配对 帧{f_prev}→帧{f_cur}  gap={f_cur - f_prev if f_prev is not None else -1}')
            if img_k is None or img_k1 is None or f_prev is None:
                self.n_drop += 1
            else:
                # ★ 单次唤醒出异常不该把整条链打死（曾因日志里取错键 KeyError 直接终止回调）
                # ★★ prev_frame 只在 step 成功后前进：失败段的运动会丢进链里
                #   （abs 是链式累积，漏一段 ⇒ 后面全部错位）。失败时保留 f_prev，
                #   下次配 (f_prev, f_next) 把失败段的跨度一起量回来。
                try:
                    ok = self.process(msg, img_k1, img_k, raw_k1, f_cur, f_prev,
                                      dep_prev=self._depth(f_prev), ns_prev=ns_prev)
                    if ok:
                        self.prev_frame = f_cur
                except Exception as e:
                    self.get_logger().error(f'唤醒 f{f_cur} 失败: {type(e).__name__}: {e}')
        self.last_recv = f_cur
        self.prev_pc_ns = ns

    # ---------------- 核心 ----------------
    def process(self, msg, img_prev, img_cur, raw_prev, frame_cur=None, frame_prev=None,
                 dep_prev=None, ns_prev=None):
        self.n_wake += 1
        self.frame_cur = frame_cur
        self.frame_prev = frame_prev
        # ★ 激光投影改为**按需**：RGB-D 直赋深命中时整段跳过（省 ~160 ms/唤醒），
        #   只有深度图缺命中时才回退到雷达投影赋深。
        lu, lz, t_proj = None, None, 0.0

        t0 = time.perf_counter()
        # 图已经是灰度（--img_pre 直读，或 grab_gray 内部转好）⇒ 这里只兜一次类型/通道
        g1 = img_prev if img_prev.ndim == 2 else cv2.cvtColor(img_prev, cv2.COLOR_BGR2GRAY)
        g2 = img_cur if img_cur.ndim == 2 else cv2.cvtColor(img_cur, cv2.COLOR_BGR2GRAY)
        t_gray = time.perf_counter() - t0

        t0 = time.perf_counter()
        plt, nf = None, 0
        if self.a.frontend == 'orb':
            # ★ ORB 描述子匹配（2026-10-27 加）：bridge1 是摆动轨迹，GT 角速度中位
            #   34°/s ⇒ 0.4 s 内转 13.6° ⇒ 图像位移 ~143 px，远超 LK 21×21/3 层的
            #   ~84 px 上限（实测 FB 存活仅 6%）。ORB 不依赖"小位移"假设，同一对帧
            #   给出 94 个几何自洽匹配（LK+lv4 只有 32），代价是 128 ms vs 50 ms。
            if self._orb is None:
                self._orb = cv2.ORB_create(self.a.orb_feat)
                self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            k1, d1 = self._orb.detectAndCompute(g1, None)
            k2, d2 = self._orb.detectAndCompute(g2, None)
            if d1 is None or d2 is None or len(k1) < 8 or len(k2) < 8:
                self.get_logger().warn(f'ORB 特征不足（{len(k1 or [])}/{len(k2 or [])}），跳过')
                return False
            ms = self._bf.match(d1, d2)
            ms = sorted(ms, key=lambda m: m.distance)[:self.a.orb_feat]
            if len(ms) < 8:
                self.get_logger().warn(f'ORB 匹配不足（{len(ms)}），跳过')
                return False
            up = np.float32([k1[m.queryIdx].pt for m in ms])
            uc = np.float32([k2[m.trainIdx].pt for m in ms])
            nf = len(k1)
        elif self.a.frontend in ('pl', 'st'):
            rm = self.plm.match(g1, g2, det=('st' if self.a.frontend == 'st' else 'fast'))
            self.n_det_thr = getattr(self.plm, '_thr_used', 0)
            up, uc = rm['uv_prev'], rm['uv_cur']
            plt, nf = rm['t'], rm['n_fast']      # ★ 另存：后面 r 会被 vo.step 覆盖
            for k, val in plt.items():
                self.acc_pl[k] = self.acc_pl.get(k, 0.0) + val
            if len(up) < 8:
                self.get_logger().warn(f'PL 特征不足（FAST {nf} → {len(up)}），跳过')
                return False
        elif self.a.frontend in ('hybrid', 'stcv2'):
            _det = 'st' if self.a.frontend == 'stcv2' else 'fast'
            # PL 检点 + cv2 三层金字塔 LK（跟踪那 6 ms 不值得上 PL，且 PL LK 只有 ±5 px）
            up, nf, plt = self.plm.detect(g1, det=_det)
            self.n_det_thr = getattr(self.plm, '_thr_used', 0)
            for k, val in plt.items():
                self.acc_pl[k] = self.acc_pl.get(k, 0.0) + val
            if len(up) < 8:
                self.get_logger().warn(f'PL FAST 检点不足（{len(up)}），跳过')
                return False
            p1, st, _ = cv2.calcOpticalFlowPyrLK(
                g1, g2, up.reshape(-1, 1, 2).astype(np.float32), None,
                winSize=(21, 21), maxLevel=3)
            st = st.reshape(-1).astype(bool)
            uc = p1.reshape(-1, 2)[st]
            up = up[st]
        else:
            p0 = cv2.goodFeaturesToTrack(g1, self.a.max_cnt, self.a.quality, self.a.min_dist)
            if p0 is None or len(p0) < 8:
                self.get_logger().warn('特征不足，跳过')
                return False
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0, None,
                                                 winSize=(21, 21), maxLevel=3)
            st = st.reshape(-1).astype(bool)
            up, uc = p0.reshape(-1, 2)[st], p1.reshape(-1, 2)[st]
        t_match = time.perf_counter() - t0

        X3_pre = None
        if DEPTH_DIR and dep_prev is not None and len(up):
            # ★ RGB-D 赋深：tracked 特征直接查 prev 帧对齐深度图，**逐点反投影成 X3**
            #   直喂 PnP（diag_fail.py C 路）。
            #   旧写法是把 (uv,z) 当激光点再过 assign_depth——它的 min_pts=2 邻域
            #   门控会把孤立特征点全部吃掉（特征间距 ≥20 px > 搜索半径 4 px），
            #   导致 X3 恒空、step 永远 None。这就是"赋深 30 但 PnP 全失败"的真因。
            ui = np.clip(np.rint(up[:, 0]).astype(int), 0, dep_prev.shape[1] - 1)
            vi = np.clip(np.rint(up[:, 1]).astype(int), 0, dep_prev.shape[0] - 1)
            zz = dep_prev[vi, ui]
            okz = np.isfinite(zz) & (zz > 0.1) & (zz < 8.0)
            if self.n_wake <= 3:
                self.get_logger().info(f'[DBG] DEPTH_DIR={bool(DEPTH_DIR)} dep_prev={"None" if dep_prev is None else dep_prev.shape} up={len(up)} okz={int(okz.sum())} n_rgbd={getattr(self, "n_rgbd", "NA")}')
            if okz.sum() >= 8:
                uq, zq = up[okz], zz[okz]
                X3_pre = np.stack([(uq[:, 0] - self.a.cx) / self.a.fx * zq,
                                   (uq[:, 1] - self.a.cy) / self.a.fx * zq,
                                   zq], 1)
                up, uc = uq, uc[okz]           # PnP 只用有深度的点对
                self.n_rgbd += 1
        if X3_pre is None:
            # 回退：雷达投影赋深（原路径）；点云懒解码后再用
            if raw_prev is None:
                m_prev = self.msg_buf.get(ns_prev) if ns_prev is not None else None
                if m_prev is not None:
                    try:
                        raw_prev = pc2_xyz(m_prev)
                    except Exception:
                        raw_prev = None
            if raw_prev is None:
                return False
            t0 = time.perf_counter()
            C = raw_prev @ self.T_v_c2[:3, :3].T + self.T_v_c2[:3, 3]
            m = C[:, 2] > 1.0
            C = C[m]
            u = self.a.fx * C[:, 0] / C[:, 2] + self.a.cx
            v = self.a.fx * C[:, 1] / C[:, 2] + self.a.cy
            ok = (u >= 0) & (u < self.a.wimg) & (v >= 0) & (v < self.a.himg)
            lu, lz = np.stack([u[ok], v[ok]], 1), C[ok, 2]
            t_proj = time.perf_counter() - t0
        t0 = time.perf_counter()
        r = self.vo.step(up, uc, lu, lz, X3_pre=X3_pre)
        t_step = time.perf_counter() - t0

        self.acc['gray'] += t_gray
        self.acc['proj'] += t_proj
        self.acc['match'] += t_match
        self.acc['step'] += t_step

        if r is None:
            self.get_logger().warn(f'#{self.n_wake} step=None（特征 {len(up)}）')
            return False
        self.n_ok += 1

        # 相对位姿 + 协方差（T_{k-1←k}，直接喂 BetweenFactor）
        Tb, cov = r['T_between'], r['cov6']
        # ★ 可选：把相对位姿从**相机系**共轭到 **body(=velo) 系**，供 map_optimization 直接当因子用。
        #   关系：T_W_C = T_W_B · T_BC ⇒ T_B0_B1 = T_BC · T_C0_C1 · T_BC⁻¹（共轭，不是简单左乘）。
        #   协方差同步用伴随矩阵变换：Σ_B = Ad(T_BC)·Σ_C·Ad(T_BC)ᵀ。
        #   放在这里做的好处：map 节点不必知道相机外参，且这一段可以离线用 GT 验证。
        if getattr(self.a, 'out_frame', 'cam') == 'velo':
            T_bc = np.linalg.inv(self.T_v_c2)          # 相机在 body 下的位姿
            Tb = T_bc @ Tb @ self.T_v_c2
            if cov is not None:
                Ad = _adjoint(T_bc)
                cov = Ad @ np.asarray(cov, float) @ Ad.T
        pr = PoseWithCovarianceStamped()
        pr.header = msg.header
        pr.pose.pose.position.x, pr.pose.pose.position.y, pr.pose.pose.position.z = \
            float(Tb[0, 3]), float(Tb[1, 3]), float(Tb[2, 3])
        q = _quat(Tb[:3, :3])
        pr.pose.pose.orientation.x, pr.pose.pose.orientation.y, \
            pr.pose.pose.orientation.z, pr.pose.pose.orientation.w = q
        if cov is not None:
            pr.pose.covariance = [float(x) for x in np.asarray(cov).reshape(-1)]
        # ★ abs 节流：map_optimization 的 vis_q_ 只保留 80 条样本。abs 5 Hz 全发时
        #   窗口只覆盖最近 16 s，而 map 处理慢（~300 ms/帧）会越落越后 ⇒ 关键帧
        #   时刻早于 q.front() ⇒ visPoseAt 全部失败 ⇒ 因子一条都加不上（实测 0 条）。
        #   每 N 条成功发一条 ⇒ N=4 时 52 s 只积 63 条 < 80 ⇒ 窗口永远覆盖全程。
        self.p_rel.publish(pr)      # rel 每条都发（诊断用；map 不订阅它）

        P = self.vo.pose()
        if getattr(self.a, 'out_frame', 'cam') == 'velo':
            # ★★ abs 也必须换到 body 系：map 侧用的是 Pa.between(Pb)，帧必须与 rel 一致。
            #   T_W_C = T_W_B · T_BC ⇒ T_W_B = T_W_C · T_BC⁻¹ = P · T_v_c2。
            #   一致性自检（离线已验证）：(Pa·T_v_c2)⁻¹(Pb·T_v_c2) ≡ T_bc·(Pa⁻¹Pb)·T_v_c2，
            #   正好等于上面 rel 的共轭结果 ⇒ 两条通道同一帧、同一量。
            P = P @ self.T_v_c2
        pa = PoseStamped()
        pa.header = msg.header
        pa.pose.position.x, pa.pose.position.y, pa.pose.position.z = \
            float(P[0, 3]), float(P[1, 3]), float(P[2, 3])
        qa = _quat(P[:3, :3])
        pa.pose.orientation.x, pa.pose.orientation.y, pa.pose.orientation.z, \
            pa.pose.orientation.w = qa
        self.n_abs_pub += 1
        if self.a.abs_every <= 1 or (self.n_abs_pub % self.a.abs_every) == 1:
            self.p_abs.publish(pa)

        tm = Float32MultiArray()
        tm.data = [1000 * t_proj, 1000 * t_match, 1000 * t_step,
                   float(r['n_inliers']), float(r['rms_px'])]
        self.p_t.publish(tm)

        # 导出累积位姿（帧号 + 4×4），用于离线与 GT 对拍（量端到端精度，不只量 step 内部）
        if self.dump is not None:
            # 格式：f_prev f_cur  <T_between 16>  <T_abs 16>
            # ★ T_between 才是给 BetweenFactor 的量；T_abs 是按帧累积的绝对位姿，
            #   降频（every>1）时中间帧没算，累积链下标不连续 ⇒ T_abs 不可用于评测。
            self.dump.write(
                f'{self.frame_prev} {self.frame_cur} '
                + ' '.join(f'{x:.9f}' for x in Tb.reshape(-1)) + ' '
                + ' '.join(f'{x:.9f}' for x in P.reshape(-1)) + '\n')
            self.dump.flush()

        if self.n_wake % 10 == 1 or self.n_wake <= 3:
            extra = ''
            if plt is not None:
                parts = ' '.join(f'{k} {plt[k]:.1f}' for k in
                                 ('pyramid', 'upload', 'fast', 'st', 'nms', 'select', 'lk')
                                 if k in plt)
                extra = f' | PL {parts}（FAST 检出 {nf}）'
            self.get_logger().info(
                f'#{self.n_wake} 特征 {len(up):3d} 赋深 {r["n_used"]:3d} 内点 {r["n_inliers"]:3d} '
                f'rms {r["rms_px"]:.2f}px |t| {r["t_norm"]:.3f}  '
                f'花费 投影 {1000*t_proj:.1f} 灰度 {1000*t_gray:.1f} 匹配 {1000*t_match:.1f} '
                f'step {1000*t_step:.1f} ms{extra}')
        return True

    def report(self):
        n = max(self.n_wake, 1)
        self.get_logger().info(
            f'汇总：收到 {self.n_pc} 帧 / 唤醒 {self.n_wake} / 成功 {self.n_ok} / 未配对 {self.n_drop} / 重锚 {self.n_reanchor}  |  '
            f'均值 投影 {1000*self.acc["proj"]/n:.1f} 灰度 {1000*self.acc["gray"]/n:.1f} '
            f'匹配 {1000*self.acc["match"]/n:.1f} step {1000*self.acc["step"]/n:.1f} ms  '
            f'合计 {1000*(self.acc["proj"]+self.acc["gray"]+self.acc["match"]+self.acc["step"])/n:.1f} ms/帧')
        if self.acc_pl:
            m = max(self.n_wake, 1)
            s = '  '.join(f'{k} {v/m:.2f}' for k, v in sorted(self.acc_pl.items()))
            self.get_logger().info(f'PL 分段均值(ms)：{s}')


def _quat(R):
    """旋转矩阵 → (x,y,z,w)，用 Shepperd 方法避免除小量。"""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return float(x), float(y), float(z), float(w)


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], float)


def _adjoint(T):
    """SE(3) 伴随矩阵，**GTSAM 的切空间顺序 [旋转(3), 平移(3)]**。

    用于把协方差从相机系搬到 body 系：若 T' = A·T·A⁻¹，则 ξ' = Ad(A)·ξ
    ⇒ Σ' = Ad(A)·Σ·Ad(A)ᵀ。顺序搞反（先平移后旋转）会让协方差被用错，
    表现是"因子权重看着对但结果偏" —— 这类错误很难从轨迹上看出来。
    """
    R, t = T[:3, :3], T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, 3:] = R
    Ad[3:, :3] = _skew(t) @ R
    return Ad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--every', type=int, default=4, help='每 N 帧点云唤醒一次（降频，板上必须）')
    ap.add_argument('--rate', type=float, default=10.0, help='仅用于打印预期唤醒率')
    ap.add_argument('--frontend', choices=('cv2', 'pl', 'hybrid', 'st', 'stcv2', 'orb'),
                    default='cv2',
                    help='匹配前端：cv2(gftt+LK) / pl(PL FAST+PL LK) / hybrid(PL FAST+cv2 LK) / '
                         'st(PL Shi-Tomasi+PL LK，忠实 LVI-SAM) / stcv2(PL Shi-Tomasi+cv2 LK)')
    ap.add_argument('--st_th', type=int, default=0,
                    help='Shi-Tomasi 绝对阈值(λ_min,0..255)；0=自适应(0.01×当帧max，等价 qualityLevel=0.01)')
    ap.add_argument('--nms', choices=('auto', 'on', 'off'), default='auto',
                    help='MIN_DIST 选点：auto=位流里有 nms_accel_0 就用硬件 / on=强制 / off=Python')
    ap.add_argument('--max_cand', type=int, default=0,
                    help='参与 MIN_DIST 抑制的候选上限（0=不限；限了省时间但丢靠后的候选）')
    ap.add_argument('--pl_bit', default='/home/ubuntu/vis3/design_1_wrapper.bit')
    ap.add_argument('--pl_th', type=int, default=20, help='PL FAST 阈值')
    ap.add_argument('--pl_levels', type=int, default=3, help='PL LK 软件金字塔层数')
    ap.add_argument('--pl_iters', type=int, default=30,
                    help='PL LK 单层迭代上限（原版 TermCriteria COUNT=30）')
    ap.add_argument('--pl_q', type=int, default=4,
                    help='flow 定点位数：4=新核 lk_lvisam(Q4,±128px) / 6=旧核 v11(Q6,±8px)')
    ap.add_argument('--max_cnt', type=int, default=60)
    ap.add_argument('--quality', type=float, default=0.01,
                    help='GFTT qualityLevel。0.05→0.01：M2DGR street1 实测 PnP 成功 '
                         '85→111/132（点更多、几何更分散）；离线 T4 也是 0.01')
    ap.add_argument('--min_dist', type=float, default=20.0)
    # ★ 内参刻意不给默认值：原默认 369.118/314.199/238.588 是 KITTI(1226×370 letterbox)
    #   遗留值，而 M2DGR-plus 是 640×480 原图、fx≈603.96 —— 差 1.64 倍。忘传时
    #   深度反投影与 PnP 会静默算错（结果坏但无任何报错），这是换数据集最常见的事故。
    ap.add_argument('--fx', type=float, default=None, help='相机 fx（必填，无默认值）')
    ap.add_argument('--cx', type=float, default=None, help='相机 cx（必填，无默认值）')
    ap.add_argument('--cy', type=float, default=None, help='相机 cy（必填，无默认值）')
    ap.add_argument('--img_pre', default=os.environ.get('IMG_PRE_DIR', ''),
                    help='预缩放灰度图目录（make_pre.py 生成）；设了则省掉在线 imread+resize(~94 ms/唤醒)')
    ap.add_argument('--wimg', type=int, default=640)
    ap.add_argument('--himg', type=int, default=480)
    ap.add_argument('--out_frame', choices=('cam', 'velo'), default='cam',
                    help='发布位姿的坐标系：cam=相机系（评测口径）；velo=body 系（喂 map_optimization 因子）')
    ap.add_argument('--depth_dir', default=os.environ.get('DEPTH_DIR', ''),
                    help='对齐深度 PNG 目录（按雷达帧序 %010d.png，16 位）；设了则 RGB-D 赋深')
    ap.add_argument('--depth_scale', type=float, default=1000.0,
                    help='深度 PNG 的毫米->米换算（M2DGR-plus=1000）')
    ap.add_argument('--tvc', default='',
                    help='velo->cam 4x4 外参 txt（覆盖 KITTI calib 组装；非 KITTI 数据必给）')
    ap.add_argument('--reproj', type=float, default=2.0)
    ap.add_argument('--orb_feat', type=int, default=300,
                    help='ORB 前端特征数（bridge1 摆动场景实测 300 ⇒ 内点 94 / 128 ms）')
    ap.add_argument('--abs_every', type=int, default=1,
                    help='abs/rel 发布节流：每 N 条成功发 1 条。map 侧 vis_q_ 只留 80 条，'
                         '5 Hz 全发会把窗口推过头导致插值全部失败（实测因子 0 条）')
    ap.add_argument('--iters', type=int, default=100)
    ap.add_argument('--pc_topic', default='/velodyne_points')
    ap.add_argument('--F0', type=int, default=0, help='回放起始帧号（本地读图用）')
    ap.add_argument('--stamps', default='',
                    help='帧时间戳表（每行一个秒，行号=图像帧号）；无帧号发布端按 stamp 查帧号')
    ap.add_argument('--dump', default=None, help='把每条累积位姿写到文件（帧号 + 16 个数/行）')
    a = ap.parse_args()
    # ★ 内参必填（见 --fx 处的说明）：宁可起不来，也不要静默用错内参跑完整条链
    if a.fx is None or a.cx is None or a.cy is None:
        ap.error('必须显式提供 --fx --cx --cy（无默认值：不同相机/数据集内参不同，'
                 '沿用默认值会静默算错而不报错；M2DGR-plus 640×480 为 fx≈603.96）')
    global IMG_PRE, DEPTH_DIR, DEPTH_SCALE
    if a.depth_dir:
        DEPTH_DIR = a.depth_dir
        DEPTH_SCALE = float(a.depth_scale)
    if a.img_pre:
        IMG_PRE = a.img_pre

    rclpy.init()
    n = VoPnp(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    except ExternalShutdownException:
        pass          # kill -INT 走的是这条；不接会打一堆 traceback，看着像崩了
    finally:
        n.report()
        if n.dump is not None:
            n.dump.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
