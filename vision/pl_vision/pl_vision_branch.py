#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lo_sam 视觉分支的下游消费者：把 PL 视觉前端接进 SLAM 的度量空间

lo_sam 本身是纯 LiDAR（无相机 / 无 IMU），架构里没有视觉分支。本节点是视觉分支的
**正式下游消费者**，做两件事：

  1) 把视觉前端输出变成"可用状态"
     - 订阅 pl_vision/flow（N×6 = [x, y, u, v, status, frame]）
     - 只用 status==1 的点。HLS 侧语义（sparse_lk.cpp:187）：
       status=1 表示迭代后跟踪点仍落在图内；**status==0 时 flow 缓冲区里是上一帧的残留值**，
       必须剔除，否则会把陈旧数据当成光流用。
     - ★ 再剔除**饱和点**（|u| >= sat_px）：本 IP 的搜索窗只有 WH=5/SEARCH=4 ⇒ 位移上限约
       ±9 px，转弯帧的真实位移会打到窗口边界，此时测到的是"窗口大小"而不是真实位移。
       把饱和点算进中位数会把 yaw 系统性压低。
     - 水平光流取**中位数**做鲁棒估计：纯前向平动的光流绕 FOE 呈反对称，中位数天然抵消；
       剩下的主导分量就是相机的**偏航(yaw)**
           Δyaw ≈ yaw_sign × (-median(u)) / fx      (fx = 640 宽下的等效焦距 ≈ 371.8 px)
       **yaw_sign = -1 是实测出来的**（与雷达航向逐帧比对，r 为负 ⇒ 取反），不靠符号推导。
     - ★ **饱和门控（sat_gate）**：单层 LK 的位移硬顶 ≈ ±7.98 px（Q6 定点 int16 的 tofix 上界），
       急转弯时真实位移远超这个量级，此时**连"未饱和"的点也被截断在 7.x**——中位数测到的是
       窗口边界，不是真实位移。实测帧 142（急弯）frac_sat=47%、p90|u|=8.0 顶格，视觉只给 −8.7°
       而雷达是 −57°（6.5× 偏小）。因此：
         frac_sat > sat_gate（默认 0.40）**或** |median(u)| > med_gate×sat_px（默认 0.9）
         ⇒ 该帧**判废、不出 yaw**（status=5，累计航向不推进、不发 vo）。
       权衡是显式的：宁可留一个**可数的覆盖缺口**（每帧丢了哪一帧的转动一目了然），
       也不要把系统性偏小的 yaw 静默入账——后者会让整条视觉轨迹永久偏斜。
     - 累积成视觉航向，发布 pl_vision/vo（nav_msgs/Odometry）
       ⚠️ 单目无深度 ⇒ **只给旋转，不给米制平移**，诚实标成 yaw-only：
       位置恒为 0、位置协方差置大值表示"平移不可用"。

  2) 与雷达前端里程计**逐帧配对**（"视觉分支真的接进 lo_sam 了"的直接证据）
     - 订阅 odom_init/laser_odom（nav_msgs/Odometry，laserOdometry 输出）
     - ★★ **必须用"碰面(rendezvous)"而不是"取最新"**：
       实测 `odom_init/laser_odom` 的 header.stamp 与 velodyne 帧时间戳 **逐条完全一致**
       （偏差 0.000 ms、帧号严格 +1），但**到达时刻有 0.4~1.4 s 的抖动滞后**
       （laserOdometry 单帧 ~575 ms，在 667 ms 节拍上排队）。
       "当前帧处理时能拿到的最新里程计"平均落后 3 帧 —— 直接取最新会全配不上或配错。
       正确做法：视觉结果按时间戳缓存，等带同一时间戳的雷达里程计到达时再配对。
     - 发布 pl_vision/vo_vs_lidar（Float64MultiArray）逐帧并排给出两路 yaw。

为什么拿 sync 当"提交信号"：视觉节点在同一次处理里先发 flow、后发 sync。
把 sync 当作"这一帧处理完了"的事件，等它到了再取该帧的 flow，就完全避开
"两个话题之间谁先到"的坑。sync.status != 0 的帧（重复/越界/读图失败）本来就没有新光流。

发布：
  pl_vision/vo          nav_msgs/Odometry     视觉 yaw-only 航向（附 yaw 角速度）
  pl_vision/vo_vs_lidar Float64MultiArray     20 列，见下方字段表
     [0] frame          相机/雷达同序号帧号
     [1] radar_stamp    本帧雷达时间戳（KITTI 原始）
     [2] n_total        flow 点数
     [3] n_valid        status==1 的点数
     [4] n_unsat        再剔掉饱和点后的可用点数
     [5] n_inlier       MAD 剔除后的内点数
     [6] med_u          水平光流中位数 (px)
     [7] med_v          垂直光流中位数 (px)
     [8] p90_abs_u      |u| 的 90 分位 (px)，用来看离窗口边界多近
     [9] frac_sat       饱和点占比 (0~1)
    [10] vis_dyaw_deg   本帧视觉偏航增量
    [11] vis_yaw_deg    视觉累计偏航
    [12] vis_rate_dps   视觉偏航角速度
    [13] lidar_yaw_deg  雷达累计偏航（相对首个配对帧）
    [14] lidar_rate_dps 雷达偏航角速度
    [15] d_yaw_deg      视觉累计 - 雷达累计
    [16] dt_ms          与上一有效帧的时间差（= KITTI 帧间隔 ~104 ms）
    [17] stamp_offset_ms 雷达里程计戳 - 本帧雷达戳（**=0 才算真正同帧**）
    [18] wall_lag_ms    雷达里程计到达 - 视觉处理完成（流水线排队滞后，正数）
    [19] status         0=配对成功 1=上游 sync 跳过 2=本地缺该帧光流
                        3=可用光流点不足 4=等雷达里程计超时未配到
                        5=饱和门控判废（frac_sat>sat_gate 或 |med_u|>med_gate×sat_px，不出 yaw）

用法（板端，需 ROS 环境；不需要 pynq）：
  python3 pl_vision_branch.py --ros-args -p fx:=371.8 -p yaw_sign:=-1.0
"""
import math
import time
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

R2D = 180.0 / math.pi
NCOL = 20
# 列名 -> 下标（便于阅读，别用魔法数字）
C_FRAME, C_STAMP, C_NTOT, C_NVAL, C_NUNSAT, C_NINL = 0, 1, 2, 3, 4, 5
C_MEDU, C_MEDV, C_P90U, C_FSAT = 6, 7, 8, 9
C_DYAW, C_VYAW, C_VRATE = 10, 11, 12
C_LYAW, C_LRATE, C_DDIFF = 13, 14, 15
C_DTMS, C_STAMPOFF, C_WALLLAG, C_STATUS = 16, 17, 18, 19


def yaw_from_quat(x, y, z, w):
    """四元数 -> 偏航角（rad）。自己算，省掉 tf_transformations 依赖"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a):
    """把角度差折回 (-pi, pi]，避免 ±180° 跳变"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def sec_to_time(t):
    s = int(math.floor(t))
    ns = int(round((t - s) * 1e9))
    if ns >= 1000000000:
        s += 1
        ns -= 1000000000
    return Time(sec=s, nanosec=ns)


class PlVisionBranch(Node):
    def __init__(self):
        super().__init__('pl_vision_branch')
        P = self.declare_parameter
        P('flow_topic', 'pl_vision/flow')
        P('sync_topic', 'pl_vision/sync')
        P('lidar_odom_topic', 'odom_init/laser_odom')
        P('vo_topic', 'pl_vision/vo')
        P('cmp_topic', 'pl_vision/vo_vs_lidar')
        P('fx', 371.8)            # 640 宽下的等效焦距（KITTI image_02 fx=721.54 x 640/1242）
        # ★ 坑：rclpy 会按 override 的字面量推断类型，命令行写 `yaw_sign:=-1` 得到 INTEGER，
        #   与这里的 DOUBLE 默认值冲突 ⇒ InvalidParameterTypeException，节点起不来。
        #   必须写 `-1.0`。这里把它翻译成看得懂的报错，免得白等一整轮流水线。
        try:
            P('yaw_sign', -1.0)   # 实测确定（见文件头说明）
        except Exception as e:    # rclpy.exceptions.InvalidParameterTypeException
            raise RuntimeError("参数 yaw_sign 类型不匹配（写成了 %r？）：命令行必须写浮点，"
                               "如 yaw_sign:=-1.0 而不是 -1。原始错误：%s" % (e, e)) from None
        P('min_valid', 20)        # 可用光流点少于此就不出 yaw
        P('mad_k', 3.0)           # MAD 剔除倍数
        # ★ 判据不是 8.0：内核的截断值实测**恰为 8.0000**（4322 个点精确落在 8.0），
        #   但紧贴硬顶的 band [7.97, 8.00) 还另有 11.6% 的点——它们同样是被窗口卡住的
        #   "假位移"，用 8.0 + 严格小于会把这一批漏掉。取 7.9 是实测扫出来的拐点：
        #   发布帧最大误差 2.80°(8.0) -> 1.10°(7.9)，再往 7.5/7.0 走只多丢帧不再降误差。
        P('sat_px', 7.9)          # |u| 达到此值视为打到搜索窗边界（截断值实测 8.0000）
        P('sat_gate', 0.40)       # 饱和点占比超过它就判废整帧（不出 yaw）；0 或负数 = 关闭门控
        P('med_gate', 0.90)       # |median(u)| > 此比例×sat_px 也判废（没饱和但已经贴着边界）
        P('stamp_tol_ms', 20.0)   # 碰面对齐的时间容差
        P('pair_timeout_s', 15.0)  # 等雷达里程计超时（超过就按"未配到"记账并丢弃）
        P('log_every', 25)
        P('send_odom', True)
        G = lambda n: self.get_parameter(n).value

        self.fx = float(G('fx'))
        self.yaw_sign = float(G('yaw_sign'))
        self.min_valid = int(G('min_valid'))
        self.mad_k = float(G('mad_k'))
        self.sat_px = float(G('sat_px'))
        self.sat_gate = float(G('sat_gate'))
        self.med_gate = float(G('med_gate'))
        self.tol = float(G('stamp_tol_ms')) * 1e-3
        self.pair_timeout = float(G('pair_timeout_s'))
        self.log_every = max(1, int(G('log_every')))
        self.send_odom = bool(G('send_odom'))

        self.flow_buf = OrderedDict()    # frame -> (N,6) ndarray
        self.vis_pend = OrderedDict()    # radar_stamp -> 视觉结果行（等雷达来配）
        self.lidar_buf = OrderedDict()   # stamp -> (yaw, recv_wall)
        self.vis_yaw = 0.0
        self.lidar_yaw_rel = 0.0
        self.lidar_prev = None
        self.prev_vis_stamp = None
        self.n_frame = 0
        self.n_ok = 0
        self.n_paired = 0
        self.n_timeout = 0
        self.n_sat_skip = 0
        self.cs = [0.0, 0.0, 0.0, 0.0, 0.0, 0]   # n, sx, sy, sxy, sxx, syy

        self.create_subscription(Float32MultiArray, G('flow_topic'), self.on_flow, 30)
        self.create_subscription(Float64MultiArray, G('sync_topic'), self.on_sync, 30)
        self.create_subscription(Odometry, G('lidar_odom_topic'), self.on_lidar, 200)
        self.pub_vo = self.create_publisher(Odometry, G('vo_topic'), 30)
        self.pub_cmp = self.create_publisher(Float64MultiArray, G('cmp_topic'), 60)

        self.get_logger().info(
            '就绪：flow=%s sync=%s lidar=%s | fx=%.1f yaw_sign=%+.0f min_valid=%d '
            'sat=%.1fpx sat_gate=%.0f%% med_gate=%.2f tol=%.0f ms'
            % (G('flow_topic'), G('sync_topic'), G('lidar_odom_topic'),
               self.fx, self.yaw_sign, self.min_valid, self.sat_px,
               self.sat_gate * 100, self.med_gate, self.tol * 1e3))

    # ---------------- 订阅回调 ----------------
    def on_flow(self, msg):
        d = np.asarray(msg.data, np.float32)
        if d.size == 0:
            self.get_logger().info('收到空光流（该帧无 LK，例如首帧）', once=True)
            return
        if d.size % 6:
            self.get_logger().warn('flow 长度不是 6 的倍数：%d，丢弃' % d.size)
            return
        a = d.reshape(-1, 6)
        if not np.all(a[:, 5] == a[0, 5]):
            self.get_logger().warn('flow 内帧号不一致，丢弃')
            return
        self.flow_buf[int(round(float(a[0, 5])))] = a
        while len(self.flow_buf) > 16:
            self.flow_buf.popitem(last=False)

    def on_lidar(self, msg):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        self.lidar_buf[st] = (yaw_from_quat(q.x, q.y, q.z, q.w), time.time())
        while len(self.lidar_buf) > 600:
            self.lidar_buf.popitem(last=False)
        self._rendezvous()

    def on_sync(self, msg):
        if len(msg.data) < 10:
            return
        v = [float(x) for x in msg.data[:10]]
        idx, radar_stamp, st_up = int(round(v[2])), v[3], int(round(v[9]))
        self.n_frame += 1

        if st_up != 0:
            self._pub_row(self._row(idx, radar_stamp, 1))
            return
        a = self.flow_buf.pop(idx, None)
        if a is None:
            self._pub_row(self._row(idx, radar_stamp, 2))
            return

        # ---- 1) status 过滤 ----
        ok = a[:, 4].astype(np.int32) == 1
        n_total, n_valid = int(a.shape[0]), int(ok.sum())
        if n_valid == 0:
            self._pub_row(self._row(idx, radar_stamp, 3, C_NTOT, n_total, C_NVAL, n_valid))
            return

        # ---- 2) 饱和点剔除（打到 ±9 px 搜索窗边界的点，测到的是窗口不是真实位移）----
        uu_all = a[ok, 2]
        unsat = np.abs(uu_all) < self.sat_px
        n_unsat = int(unsat.sum())
        frac_sat = 1.0 - (n_unsat / float(n_valid))
        p90 = float(np.percentile(np.abs(uu_all), 90))
        base = self._row(idx, radar_stamp, 3, C_NTOT, n_total, C_NVAL, n_valid,
                         C_NUNSAT, n_unsat, C_P90U, p90, C_FSAT, frac_sat)

        # ---- 2b) 饱和门控：整片位移已超出 ±8 px 硬顶，测到的是窗口边界而不是真实位移 ----
        #   帧 142 实测 frac_sat=47%、p90|u|=8.0 顶格 ⇒ 视觉 −8.7° vs 雷达 −57°（6.5× 偏小）。
        #   判废（status=5）：不推进累计航向、不发 vo，只把这一帧的 frac_sat/p90/med_u 记进账。
        if self.sat_gate > 0:
            med_u_all = float(np.median(uu_all))
            if frac_sat > self.sat_gate or abs(med_u_all) > self.med_gate * self.sat_px:
                self.n_sat_skip += 1
                base[C_MEDU] = med_u_all
                base[C_STATUS] = 5.0
                # 时间照走：下一帧的 dt 不被这个缺口拉长，角速度不会被稀释
                self.prev_vis_stamp = radar_stamp
                self._pub_row(base)
                if self.n_sat_skip <= 3 or self.n_sat_skip % self.log_every == 0:
                    self.get_logger().warn(
                        '饱和判废 frame %d | 饱和 %.0f%% (>%.0f%%) med_u %+.2f px p90|u| %.2f px '
                        '| 该帧不出 yaw（累计 %d 帧）'
                        % (idx, frac_sat * 100, self.sat_gate * 100, med_u_all, p90,
                           self.n_sat_skip))
                return

        if n_unsat < self.min_valid:
            self._pub_row(base)
            return

        # ---- 3) 中位数 + MAD 剔除 ----
        uu = uu_all[unsat]
        vv = a[ok, 3][unsat]
        med_u = float(np.median(uu))
        mad = float(np.median(np.abs(uu - med_u)))
        gate = self.mad_k * mad if mad > 1e-6 else 1e6
        inl = np.abs(uu - med_u) <= gate
        n_inl = int(inl.sum())
        med_u_in = float(np.median(uu[inl])) if n_inl else med_u
        med_v = float(np.median(vv))

        dt = (radar_stamp - self.prev_vis_stamp) if self.prev_vis_stamp is not None else float('nan')
        d_yaw = self.yaw_sign * (-med_u_in / self.fx)          # rad / 帧
        self.vis_yaw += d_yaw
        vis_rate = (d_yaw * R2D / dt) if (dt == dt and dt > 0) else float('nan')
        self.prev_vis_stamp = radar_stamp
        self.n_ok += 1

        if self.send_odom:
            od = Odometry()
            od.header.stamp = sec_to_time(radar_stamp)
            od.header.frame_id = 'vision_vo'
            od.child_frame_id = 'camera'
            od.pose.pose.orientation.z = math.sin(self.vis_yaw / 2.0)
            od.pose.pose.orientation.w = math.cos(self.vis_yaw / 2.0)
            od.pose.covariance[0] = 1e6      # x 不可用（单目无深度）
            od.pose.covariance[7] = 1e6      # y 不可用
            od.pose.covariance[14] = 1e6     # z 不可用
            od.pose.covariance[35] = max(1e-8, (mad / self.fx) ** 2)
            od.twist.twist.angular.z = (d_yaw / dt) if (dt == dt and dt > 0) else 0.0
            self.pub_vo.publish(od)

        base[C_NINL] = n_inl
        base[C_MEDU] = med_u
        base[C_MEDV] = med_v
        base[C_DYAW] = d_yaw * R2D
        base[C_VYAW] = self.vis_yaw * R2D
        base[C_VRATE] = vis_rate
        base[C_DTMS] = (dt * 1e3) if dt == dt else float('nan')
        base[C_STATUS] = float('nan')        # 等配对后再定 status
        base.append(time.time())             # 第 20 位：视觉处理完成时刻（不发布）
        self.vis_pend[radar_stamp] = base
        self._rendezvous()

    # ---------------- 碰面对齐 ----------------
    def _rendezvous(self):
        """把 vis_pend 与 lidar_buf 里【时间戳相同】的两条配成一对

        逐条尝试（待配条数很少）。配不上的两种情况：
          a) 雷达已越过这一帧且超过排队抖动上限 -> 永远配不上，记 status=4 后丢弃
          b) 等太久（pair_timeout_s）-> 同样记 status=4 后丢弃
        这样既不会漏配，也不会被一条配不上的记录把队列堵死。
        """
        if not self.vis_pend:
            return
        newest_lidar = max(self.lidar_buf) if self.lidar_buf else None
        for stamp in list(self.vis_pend.keys()):
            row = self.vis_pend[stamp]
            ls, ly, _d = self._nearest_lidar(stamp)
            if ls is not None:
                del self.vis_pend[stamp]
                self._emit_pair(row, ls, ly)
                continue
            passed = newest_lidar is not None and (newest_lidar - stamp) > 1.0
            if passed or (time.time() - row[NCOL] > self.pair_timeout):
                del self.vis_pend[stamp]
                self.n_timeout += 1
                row[C_STATUS] = 4.0
                self._pub_row(row[:NCOL])

    def _nearest_lidar(self, stamp):
        best_s, best_y, best_d = None, None, None
        for s, (y, _t) in self.lidar_buf.items():
            d = abs(s - stamp)
            if best_d is None or d < best_d:
                best_s, best_y, best_d = s, y, d
        if best_s is None or best_d > self.tol:
            return None, None, None
        return best_s, best_y, best_d

    def _emit_pair(self, row, ls, ly):
        """把配对上的雷达偏航填进这一行并发布"""
        radar_stamp = row[C_STAMP]
        lidar_recv = self.lidar_buf[ls][1]
        if self.lidar_prev is None:
            lidar_rel = 0.0
        else:
            dt = ls - self.lidar_prev[0]
            dy = wrap_pi(ly - self.lidar_prev[1])
            row[C_LRATE] = (dy * R2D / dt) if dt > 0 else float('nan')
            self.lidar_yaw_rel += dy
            lidar_rel = self.lidar_yaw_rel
        self.lidar_prev = (ls, ly)
        lidar_deg = lidar_rel * R2D
        self.n_paired += 1

        row[C_LYAW] = lidar_deg
        row[C_DDIFF] = row[C_VYAW] - lidar_deg
        row[C_STAMPOFF] = (ls - radar_stamp) * 1e3
        row[C_WALLLAG] = (lidar_recv - row[NCOL]) * 1e3
        row[C_STATUS] = 0.0

        # 相关系数累加（视觉角速度 vs 雷达角速度）
        vr, lr = row[C_VRATE], row[C_LRATE]
        if vr == vr and lr == lr:
            n, sx, sy, sxy, sxx, syy = self.cs
            self.cs = [n + 1, sx + vr, sy + lr, sxy + vr * lr,
                       sxx + vr * vr, syy + lr * lr]

        self._pub_row(row[:NCOL])

        if self.n_paired % self.log_every == 0 or self.n_paired <= 3:
            self.get_logger().info(
                '配对#%d frame %d | 点 %d/有效 %d/可用 %d/内点 %d 饱和 %.0f%% p90|u| %.1f px '
                '| 视觉 %+7.2f° (%.2f°/s) | 雷达 %+7.2f° (%.2f°/s) | 差 %+6.2f° '
                '| 戳偏 %.3f ms 排队滞后 %.0f ms | r=%.3f'
                % (self.n_paired, int(row[C_FRAME]), int(row[C_NTOT]), int(row[C_NVAL]),
                   int(row[C_NUNSAT]), int(row[C_NINL]), row[C_FSAT] * 100, row[C_P90U],
                   row[C_VYAW], row[C_VRATE], row[C_LYAW], row[C_LRATE], row[C_DDIFF],
                   row[C_STAMPOFF], row[C_WALLLAG], self._corr()))

    def _corr(self):
        n, sx, sy, sxy, sxx, syy = self.cs
        if n < 3:
            return float('nan')
        cov = sxy / n - (sx / n) * (sy / n)
        vx = sxx / n - (sx / n) ** 2
        vy = syy / n - (sy / n) ** 2
        if vx <= 1e-12 or vy <= 1e-12:
            return float('nan')
        return cov / math.sqrt(vx * vy)

    # ---------------- 工具 ----------------
    def _row(self, idx, radar_stamp, status, *kv):
        r = [float('nan')] * NCOL
        r[C_FRAME] = float(idx)
        r[C_STAMP] = float(radar_stamp)
        r[C_STATUS] = float(status)
        for i in range(0, len(kv), 2):
            r[int(kv[i])] = float(kv[i + 1])
        return r

    def _pub_row(self, row):
        m = Float64MultiArray()
        m.data = [float(x) for x in row[:NCOL]]
        self.pub_cmp.publish(m)


def main():
    rclpy.init()
    node = PlVisionBranch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        lg = node.get_logger()
        lg.info('收尾：sync %d 帧，成功估计 %d，配对 %d，超时未配 %d，饱和判废 %d'
                % (node.n_frame, node.n_ok, node.n_paired, node.n_timeout,
                   node.n_sat_skip))
        lg.info('视觉 yaw 累计 %+.2f° | 雷达 yaw 累计 %+.2f° | 逐帧角速度相关系数 r=%.3f'
                % (node.vis_yaw * R2D, node.lidar_yaw_rel * R2D, node._corr()))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
