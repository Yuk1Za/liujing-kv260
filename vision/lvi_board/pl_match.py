#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pl_match —— PL(FPGA) 视觉匹配前端：FAST 检角点 + 稀疏 LK 金字塔跟踪。

替代 `vo_pnp_node.py` 里 cv2 `goodFeaturesToTrack + calcOpticalFlowPyrLK` 那一段。

为什么需要「软件金字塔」
------------------------
`sparse_lk` IP 是**单层**核：窗口 11×11、片上 Cb patch 只有 21×21，
**单次调用能细化的残差最多约 ±5 px**（再大就读到 patch 外），输出定点 tofix 也把
流钳在 ±8 px。而板上实测 KITTI 0027 帧间光流**中位 8.4 px、p90 24.5 px、最大 >150 px**
⇒ 直接喂纯 LK 只能跟踪到极小位移的特征。

IP 头部注释写的调用契约就是「单层 + 初值；金字塔由软件从粗到细编排」，本文件实现：
    第 l 层用 `flow/2^l` 当初值 → 硬件在局部细化 → 结果 ×2^l 带回全分辨率
逐层累加后捕获范围 ≈ 8·(2^L−1) px（L=3 ⇒ 56 px，够覆盖 p90=24.5）。

寄存器（一律裸 mmio；`rm.<name>` 在这些 hwh 上会**静默假成功**）
    FAST  AP 0x00 | img_in 0x10 | threshold 0x1c | img_out 0x24 | rows 0x30 | cols 0x38
    SLK   AP 0x00 | prev 0x10 | cur 0x1c | corners 0x28 | flow 0x34 | status 0x40
          N 0x4c | rows 0x54 | cols 0x5c | max_iter 0x64

调用契约（v11 核）
------------------
`corners` / `flow_io` / `status` 三个数组各需**多留 3 个元素**（尾组整组读写）。
`flow_io` 是 in/out：输入 Q6 定点初值（u 高 16 位、v 低 16 位），输出细化后的流。
`corners` 必须是 **prev 帧坐标** `(y<<16)|x`。

两条踩过的坑
------------
1. **每层必须把 S_PREV/S_CUR 指到该层在 buffer 里的偏移**。漏了 ⇒ 粗层仍读第 0 层开头，
   正好是 letterbox 的黑边，梯度全 0 ⇒ det<MIN_DET ⇒ **整层 tracked=0**（现象：粗层全灭、
   细层正常）。
2. FAST 输出是**整幅写**（0/255），不需要预清；但要按 **uint32 整字**判零再解包，
   直接对 307,200 字节做 nonzero 在 A53 上要 6–7 ms。

对外
----
    m = PLMatcher(max_cnt=60)
    r = m.match(prev_gray, cur_gray)      # 两帧都必须是 (480,640) uint8 单通道
    r['uv_prev'], r['uv_cur'], r['ok']    # 与 cv2 分支同形状
    r['t']                                # 各段耗时（ms）
"""
import time

import numpy as np

F_AP, F_IN, F_TH, F_OUT, F_R, F_C = 0x00, 0x10, 0x1c, 0x24, 0x30, 0x38
S_AP, S_PREV, S_CUR, S_COR, S_FLOW, S_ST = 0x00, 0x10, 0x1c, 0x28, 0x34, 0x40
S_N, S_ROWS, S_COLS, S_ITER = 0x4c, 0x54, 0x5c, 0x64
# Shi-Tomasi（shitomasi_accel_0）。⚠ 与 FAST **布局不同**（参数顺序不同），必须按名字写
#   img_in/_out 各占 8 字节（_1=低 32 位 @+0，_2=高 32 位 @+4）
T_AP, T_IN, T_OUT = 0x00, 0x10, 0x1c
T_ROWS, T_COLS, T_TH, T_K = 0x28, 0x30, 0x38, 0x40
# MIN_DIST NMS（nms_accel_0）：把「降序 → 抑制 → 前 N」整段搬进 PL。
# ★ 偏移来自 IP 的 component.xml（**单一口**，s_axi_control）；写成 `bundle=control` 会拆成两个口。
#   resp/out_xy/out_n 各占 8 字节（_1=低 32 位 / _2=高 32 位）
N_AP, N_RESP, N_OXY, N_ON = 0x00, 0x10, 0x1c, 0x28
N_ROWS, N_COLS, N_TH, N_MD, N_BR, N_MAXN = 0x34, 0x3c, 0x44, 0x4c, 0x54, 0x5c

# 流的定点小数位。★ 两代核不同：
#   旧 sparse_lk_v11 用 Q6（FIX_SCALE=64）⇒ int16 只能表示 ±7.98 px，位移一超就饱和；
#   新 lk_lvisam（忠实 LVI-SAM 21×21）用 **Q4**（FIX_SCALE=16）⇒ ±128 px、分辨率 0.0625 px。
#   ⇒ 位流换了必须同时换这个值，否则流被放大/缩小 4 倍。
Q = 4
PMAX = 10             # 核内边界余量 = WHALF(5) + SEARCH(4) + 1

# 位流可用环境变量覆盖：PL_BIT=/home/ubuntu/vis3/design_1_wrapper_lkv2.bit
import os as _os
BIT = _os.environ.get('PL_BIT', '/home/ubuntu/vis3/design_1_wrapper.bit')
PYNQ_SP = '/usr/local/share/pynq-venv/lib/python3.10/site-packages'

# ★ 同一个 .bit 只加载一次并全局共享：重复 Overlay() 会**重下位流并把先前实例写好的
#   寄存器全部清掉**（现象：多开几个 matcher 后只有最后一个能跑，其余恒定 0 点跟踪）。
_OL = {}


def _overlay(bit):
    if bit not in _OL:
        import os
        import sys
        if PYNQ_SP not in sys.path:
            sys.path.insert(0, PYNQ_SP)     # 系统 python3 里没有 pynq，指到 pynq-venv
        if not os.access('/dev/mem', os.W_OK):
            raise RuntimeError('需要 root（加载位流要写 clock 寄存器）⇒ 用 sudo 跑')
        # ★ 缓存真实路径在 pynq/pl_server/ 下（不是 /var/lib/pynq/，我一开始写错了）：
        #   换位流后不清它，PYNQ 会沿用上一颗 bit 的 ip_dict，报
        #   "Could not find IP or hierarchy fast_accel_0 ... Did you mean 'slk_accel_0'"
        #
        # ★★ 但**共存场景不能清**：雷达节点已用同一路径加载过位流，清缓存会逼 PYNQ 重新
        #    解析/下载（风险：清掉雷达侧寄存器的既有配置）。默认只在**单独跑视觉**时清；
        #    共存时设 PL_CLEAR_CACHE=0（部署完新位流后清一次即可）。
        if os.environ.get('PL_CLEAR_CACHE', '1') == '1':
            _PSP = '/usr/local/share/pynq-venv/lib/python3.10/site-packages/pynq/pl_server'
            for f in (f'{_PSP}/_current_metadata.pkl', f'{_PSP}/global_pl_state.json',
                      '/var/lib/pynq/_current_metadata.pkl', '/var/lib/pynq/global_pl_state.json'):
                if os.path.exists(f):
                    try:
                        os.remove(f)
                    except OSError:
                        pass
        from pynq import Overlay
        # ★★ `download=False`：共存时雷达节点已经把位流加载好了，**视觉节点绝不能再来一次**
        #   下载/重定位。两次 Overlay(download=True) 会在雷达 DMA 在飞的时候重编程 PL，
        #   AXI/HP 端口死锁 ⇒ 整板挂死（现象：ping 全丢、无内核日志、必须断电）。
        #   之前验证过的共存 launch 用的就是 `download:=false`。
        #   单独跑视觉时仍用默认 True（自己负责加载）。
        dl = os.environ.get('PL_DOWNLOAD', '1') == '1'
        _OL[bit] = Overlay(bit, download=dl)
    return _OL[bit]


def _wr64(mm, a, v):
    mm.write(a, v & 0xFFFFFFFF)
    mm.write(a + 4, (v >> 32) & 0xFFFFFFFF)


def select_min_dist(resp, max_corners, min_dist, max_cand=0):
    """响应图 → 前 max_corners 个点，**逐条复刻 `cv2.goodFeaturesToTrack` 的后处理**。

    LVI-SAM 原版前端就是 `goodFeaturesToTrack(img, MAX_CNT, 0.01, MIN_DIST, mask)`：
    PL 只负责出 λ_min 响应图，排序 / MIN_DIST 抑制 / 取前 N 这三步必须由 CPU 补。
    规则是**欧氏**距离 dx²+dy² < min_dist²（严格小于），不是方形窗口 ——
    按轴对齐方形实现时，切比雪夫 19 / 欧氏 26.9 的一对点会多抑制一个，
    同响应同阈值下与 cv2 的 F1 只有 0.79–0.88。

    性能（KV260 A53 实测，都在 `pl_vision_node.py` 里验过）：
      · `np.nonzero` 在 480×640 上 6.4 ms，`cv2.findNonZero` 只要 1.24 ms ⇒ 必须用后者；
      · 逐点掩码类实现（bytearray 圆盘 27 ms / 扁平 cell 21 ms / dict 网格 23 ms /
        cv2.rectangle 11 ms）全部劣于**向量化贪心**（每轮取第一个未屏蔽候选 + 一次整数组算抑制集）；
      · `max_cand` 是唯一的速度旋钮，代价是砍掉排名靠后的候选（= 抬高阈值）。

    返回 (pts[N,2] int32 (x,y), n_cand, n_drop)
    """
    import cv2
    if max_corners <= 0:
        return np.zeros((0, 2), np.int32), 0, 0
    pos = cv2.findNonZero(resp)
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
    PX, PY = P[:, 0].astype(np.int32), P[:, 1].astype(np.int32)
    r2 = int(min_dist) * int(min_dist)
    blocked = np.zeros(n_cand, bool)
    kept = []
    for _ in range(n_take):
        i = int(np.argmin(blocked))
        if blocked[i]:
            break
        kept.append(i)
        dx, dy = PX - PX[i], PY - PY[i]
        blocked |= (dx * dx + dy * dy) < r2
    pts = P[kept].astype(np.int32)
    return pts, n_cand, n_cand - len(kept)


def select_min_dist_fast(resp, max_corners, min_dist, h=480, w=640):
    """与 `select_min_dist` **等价**（逐点相同）但快一个数量级的实现。

    为什么需要它：原实现的成本 ≈ n_cand × max_corners（每轮对整条候选数组算一次抑制集），
    实测 1451 候选 13 ms。而 Shi-Tomasi 在「自适应阈值 0.01×max」下候选有 **~2900 个**
    ⇒ 光选点就要 25 ms，把整个匹配预算吃光。

    改法：把「抑制集」从候选数组搬到**像素网格**上 —— 维护一张 (H,W) 的 bool 屏蔽图，
    每选中一个点就把半径 < min_dist 的圆盘（严格小于，与 cv2 同规则）打上标记；
    下一个候选只看自己那一个像素被没被标。每轮代价 = O(候选数) 的一次 gather + O(πr²) 的散写，
    与 min_dist 无关、与已选点数无关 ⇒ 60 点固定 ~2–3 ms。

    语义与 greedy 完全一致：候选按响应降序、严格取第一个未被抑制者，
    所以结果与 `select_min_dist` 逐点相同（chk_st.py 里有对拍）。
    """
    import cv2
    if max_corners <= 0:
        return np.zeros((0, 2), np.int32), 0, 0
    pos = cv2.findNonZero(resp)
    if pos is None:
        return np.zeros((0, 2), np.int32), 0, 0
    pos = pos.reshape(-1, 2)
    n_cand = len(pos)
    order = np.argsort(-resp[pos[:, 1], pos[:, 0]].astype(np.int32), kind='stable')
    P = pos[order]
    PX, PY = P[:, 0].astype(np.int32), P[:, 1].astype(np.int32)

    r = int(min_dist)
    if r <= 0:
        k = min(int(max_corners), n_cand)
        return P[:k].astype(np.int32), n_cand, n_cand - k
    blocked = np.zeros((h, w), bool)
    dy, dx = np.mgrid[-r:r + 1, -r:r + 1]
    keep = (dx * dx + dy * dy) < r * r          # 严格小于 = cv2 规则
    dys, dxs = dy[keep].ravel(), dx[keep].ravel()

    taken = np.empty(min(int(max_corners), n_cand), np.int64)
    nt = 0
    for _ in range(len(taken)):
        avail = ~blocked[PY, PX]
        if not avail.any():
            break
        i = int(avail.argmax())                 # 降序里第一个未被抑制的
        taken[nt] = i
        nt += 1
        yy, xx = PY[i] + dys, PX[i] + dxs
        ok = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
        blocked[yy[ok], xx[ok]] = True
    pts = P[taken[:nt]].astype(np.int32)
    return pts, n_cand, n_cand - nt


def select_min_dist_k(resp, max_corners, min_dist, k=600):
    """响应图 → 前 max_corners 个点：**按响应截断候选 + 向量化贪心**（板上最快的组合）。

    为什么长这样（都是板上实测逼出来的，别凭 x86 直觉改）：
      · 参考实现（`select_min_dist`）每轮对**整条候选数组**算一次抑制集 ⇒ 成本 ≈ n_cand × max_corners。
        板上 1300 候选 9 ms、2800 候选 10.7 ms。
      · 「像素网格屏蔽」版在 x86 上快 13×，但在 **A53 上反而慢 3×**（27 ms）：
        每轮要散写 π·min_dist² ≈ 1257 个像素，ARM 上这是纯开销。**板端只认实测。**
      · 真正的廉价杠杆是**先截断**：候选本来就已经按响应降序排好，
        取前 k 个即可（`P = pos[order][:k]`），greedy 的数组规模从 2500 降到 600 ⇒ 时间线性降。
        代价是「比第 k 名更弱的候选」不再参与 —— 但 min_dist=20 时 60 个点只用得到最强的那批，
        k=600 时点数与全量一致（60 vs 60），点集重合 cv2 仍在 93% 量级。

    语义仍与 cv2 一致：降序 + 「第一个未被抑制者」+ 欧氏严格小于。
    返回 (pts, n_cand, n_drop)；n_cand 是**截断前**的候选总数（用于诊断阈值是否合适）。
    """
    import cv2
    if max_corners <= 0:
        return np.zeros((0, 2), np.int32), 0, 0
    pos = cv2.findNonZero(resp)
    if pos is None:
        return np.zeros((0, 2), np.int32), 0, 0
    pos = pos.reshape(-1, 2)
    n_all = len(pos)
    order = np.argsort(-resp[pos[:, 1], pos[:, 0]].astype(np.int32), kind='stable')
    if k and n_all > k:
        order = order[:k]
    P = pos[order]
    n = len(P)
    n_take = min(int(max_corners), n)
    if n_take <= 0:
        return np.zeros((0, 2), np.int32), n_all, 0
    if min_dist <= 0:
        return P[:n_take].astype(np.int32), n_all, n_all - n_take
    PX = P[:, 0].astype(np.int32)
    PY = P[:, 1].astype(np.int32)
    r2 = int(min_dist) * int(min_dist)
    blocked = np.zeros(n, bool)
    taken = np.empty(n_take, np.int64)
    nt = 0
    for _ in range(n_take):
        i = int(np.argmin(blocked))          # 降序 ⇒ 「第一个未屏蔽」与「最小下标」等价
        if blocked[i]:
            break
        taken[nt] = i
        nt += 1
        dx = PX - PX[i]
        dy = PY - PY[i]
        blocked |= (dx * dx + dy * dy) < r2
    pts = P[taken[:nt]].astype(np.int32)
    return pts, n_all, n_all - nt


def select_min_dist_circle(resp, max_corners, min_dist, k=600, h=480, w=640):
    """响应图 → 前 max_corners 个点：**用 `cv2.circle` 在 C 里画抑制圆盘**（板上最快）。

    为什么这样最快：瓶颈不是算术而是 **Python↔numpy 的每次调用开销**（A53 上每次 ~10 µs）。
      · 参考实现每选一点要对整条候选数组做 6–8 次 numpy 调用 ⇒ 60 点 ≈ 8–10 ms；
      · 「按响应截断」只把数组规模变小，调用次数不变 ⇒ 只省 2 ms；
      · 这里把「抑制集」整个交给 C：`cv2.circle(mask, p, r, 1, -1)` 一次调用画好圆盘，
        每轮只剩「取一个未被抑制的候选」（1 次 gather + argmax）⇒ 60 点 ≈ 2–3 ms。

    ★ 为什么 `r = min_dist − 1` 恰好等价：cv2 的规则是「欧氏距离 d < min_dist 就抑制」，
      整数坐标下 d < min_dist ⟺ d ≤ min_dist − 1（因为 (min_dist−1)² < min_dist² < min_dist²+
      且下一档 (min_dist)² 不满足）。所以半径 min_dist−1 的圆盘与 cv2 的抑制集**逐像素相同**。
    """
    import cv2
    if max_corners <= 0:
        return np.zeros((0, 2), np.int32), 0, 0
    pos = cv2.findNonZero(resp)
    if pos is None:
        return np.zeros((0, 2), np.int32), 0, 0
    pos = pos.reshape(-1, 2)
    n_all = len(pos)
    order = np.argsort(-resp[pos[:, 1], pos[:, 0]].astype(np.int32), kind='stable')
    if k and n_all > k:
        order = order[:k]
    P = pos[order]
    PX = P[:, 0].astype(np.int32)
    PY = P[:, 1].astype(np.int32)
    n_take = min(int(max_corners), len(P))
    if n_take <= 0:
        return np.zeros((0, 2), np.int32), n_all, 0
    if min_dist <= 0:
        return P[:n_take].astype(np.int32), n_all, n_all - n_take
    r = int(min_dist) - 1
    mask = np.zeros((h, w), np.uint8)
    taken = np.empty(n_take, np.int64)
    nt = 0
    for _ in range(n_take):
        avail = mask[PY, PX] == 0
        if not avail.any():
            break
        i = int(avail.argmax())
        taken[nt] = i
        nt += 1
        cv2.circle(mask, (int(PX[i]), int(PY[i])), r, 1, -1)
    pts = P[taken[:nt]].astype(np.int32)
    return pts, n_all, n_all - nt


class PLMatcher:
    def __init__(self, bit=BIT, th=20, max_cnt=60, min_dist=20.0,
                 levels=3, max_iter=30, w=640, h=480, q=Q,
                 st_th=0, max_cand=0, sel_fast=False, sel_k=600, sel_mode='k', nms='auto'):
        self.th, self.max_cnt, self.min_dist = th, int(max_cnt), float(min_dist)
        self.levels, self.max_iter = levels, max_iter
        self.q = int(q)          # 4=新核(lk_lvisam) / 6=旧核(v11)
        self.st_th = int(st_th)  # 0 = 自适应（0.01×max，等价 cv2 qualityLevel=0.01）
        self.max_cand = int(max_cand)
        self.sel_fast = bool(sel_fast)   # ST 选点用网格屏蔽版（x86 快，但 A53 上反而慢 3×）
        self.sel_k = int(sel_k)          # ST 选点先按响应截断到前 k 个候选
        # ★ 默认 'k'：板上实测「截断 + numpy 贪心」7.8 ms 是最快且与 cv2 同语义的；
        #   'circle'（cv2.circle 画圆盘）在板上既不更快（8.8–12.8 ms）也不逐点等价，已证伪；
        #   'grid'（像素屏蔽散写）在 x86 快 13× 但在 A53 上慢 3×（27 ms）。
        self.sel_mode = str(sel_mode)    # k / circle / grid / ref
        self.nms = str(nms)              # auto/on：位流里有 nms_accel_0 就用硬件选点
        self.W, self.H = w, h
        self.t = {}
        self.n_call = 0
        self._init_hw(bit)

    # ---------------------------------------------------------------- 硬件
    def _init_hw(self, bit):
        self.ol = _overlay(bit)          # ★ 先取 overlay（它负责把 pynq-venv 加进 sys.path）
        import pynq
        from pynq import allocate
        self.ips = sorted(self.ol.ip_dict)
        # ★ 不同位流里 IP 组合不同（FAST+LK / 雷达+ST+LK / …）⇒ 按实际清单取，缺了不报错，
        #   等真正要用那个检点器时再抛（这样同一个类能同时服务 --frontend fast/st/hybrid）
        self.fm = self.ol.fast_accel_0.mmio if 'fast_accel_0' in self.ips else None
        # ★ LK 不是必需的：融合位流（雷达+ST+NMS，见 merge_fusion.tcl）里没有 slk_accel_0，
        #   而 `stcv2` 前端本来就用 cv2 的 LK（比 PL LK 又准又快）。所以在真正跑 LK 时才检查。
        self.sm = self.ol.slk_accel_0.mmio if 'slk_accel_0' in self.ips else None
        self.tm = self.ol.shitomasi_accel_0.mmio if 'shitomasi_accel_0' in self.ips else None
        self.nm = self.ol.nms_accel_0.mmio if 'nms_accel_0' in self.ips else None
        self.ver = getattr(pynq, '__version__', '?')

        # ---- 各层尺寸与在「一个大 buffer」里的偏移 ----
        # 一层一次 sync_to_device 太贵（3 层 × 2 图 = 6 次小 DMA）；拼成一块只 sync 2 次
        self.dims = []
        for l in range(self.levels):
            s = 1 << l
            hh, ww = self.H // s, self.W // s
            if ww % 4:
                raise ValueError(f'层 {l} 宽 {ww} 不是 4 的倍数')
            self.dims.append((hh, ww, hh * ww // 4))
        self.off, acc = [], 0
        for _, _, nw in self.dims:
            self.off.append(acc)
            acc += nw
        self.pb = allocate(shape=(acc,), dtype=np.uint32, cacheable=1)
        self.cb = allocate(shape=(acc,), dtype=np.uint32, cacheable=1)
        self.fout = allocate(shape=(self.H * self.W // 4,), dtype=np.uint32, cacheable=1)

        # corner / flow / status 合成一块：每层从 5 次 sync 降到 2 次
        cap = ((self.max_cnt + 16 + 15) // 16) * 16       # 16 字对齐（64 B）
        self.cap = cap
        self.cfs = allocate(shape=(3 * cap,), dtype=np.uint32, cacheable=1)
        base = self.cfs.device_address
        self.a_cor, self.a_flw, self.a_sta = base, base + 4 * cap, base + 8 * cap

        # NMS 的输出：64 个角点字 + 1 个字（点数 | 溢出<<31）
        self.nbuf = None
        # ST 的响应图 buffer：(H,W) uint8，按 uint32 看就是 H*W/4 字
        self.rbuf = None
        if self.tm is not None:
            self.rbuf = allocate(shape=(self.H * self.W // 4,), dtype=np.uint32, cacheable=1)
        if self.nm is not None:
            self.nbuf = allocate(shape=(self.max_cnt + 8,), dtype=np.uint32, cacheable=1)
            cap = int(self.max_cnt) + 8
            _wr64(self.nm, N_OXY, self.nbuf.device_address)
            _wr64(self.nm, N_ON, self.nbuf.device_address + 4 * (cap - 1))
            ap = self.nm.read(N_AP)
            if not ((ap >> 2) & 1):
                raise RuntimeError(f'NMS IP 不在 idle: AP=0x{ap:08X}')

        if self.fm is not None:
            _wr64(self.fm, F_IN, self.pb.device_address)      # 第 0 层偏移 0
            _wr64(self.fm, F_OUT, self.fout.device_address)
            # ★ 写-回读确认通路真的通了（只读 0 无法区分"没寄存器"和"值就是 0"）
            self.fm.write(F_TH, self.th)
            if self.fm.read(F_TH) != self.th:
                raise RuntimeError(f'FAST threshold 回读不符: {self.fm.read(F_TH)}')
            ap = self.fm.read(F_AP)
            if not ((ap >> 2) & 1):
                raise RuntimeError(f'FAST IP 不在 idle: AP=0x{ap:08X}')
        if self.tm is not None:
            _wr64(self.tm, T_IN, self.pb.device_address)
            _wr64(self.tm, T_OUT, self.rbuf.device_address)
            self.tm.write(T_ROWS, self.H)
            self.tm.write(T_COLS, self.W)
            if self.tm.read(T_ROWS) != self.H:
                raise RuntimeError('Shi-Tomasi rows 回读不符')
            ap = self.tm.read(T_AP)
            if not ((ap >> 2) & 1):
                raise RuntimeError(f'Shi-Tomasi IP 不在 idle: AP=0x{ap:08X}')

    # ---------------------------------------------------------------- 工具
    def _upload(self, buf, pyr):
        for l, (hh, ww, nw) in enumerate(self.dims):
            buf[self.off[l]:self.off[l] + nw] = \
                np.ascontiguousarray(pyr[l]).view(np.uint32).reshape(-1)
        buf.sync_to_device()

    def _fast_points(self):
        """读 FAST 掩码 → 角点 (x,y)。按 uint32 整字先判零，再解包命中的字节。"""
        w = np.asarray(self.fout)                  # uint32，76800 字（无拷贝）
        nz = np.flatnonzero(w)                     # 只有 ~1e3 个字非零
        if nz.size == 0:
            return np.zeros((0, 2))
        b = w[nz].view(np.uint8).reshape(-1, 4)    # (k,4) 每字节对应一个像素
        k, j = np.nonzero(b)
        idx = nz[k].astype(np.int64) * 4 + j
        ys, xs = np.divmod(idx, self.W)
        return np.stack([xs, ys], 1).astype(np.float64)

    def _select(self, pts):
        """边界过滤 + 网格去重 + 均匀抽稀到 max_cnt（FAST 无响应值，不能按强度排序）。"""
        m = ((pts[:, 0] >= PMAX) & (pts[:, 1] >= PMAX) &
             (pts[:, 0] < self.W - PMAX) & (pts[:, 1] < self.H - PMAX))
        p = pts[m]
        if len(p) <= self.max_cnt:
            return p
        # 先粗抽稀，把 np.unique 的规模压到 ~4×max_cnt（否则 1100 个点上要 1.5 ms）
        k = int(np.ceil(len(p) / (4.0 * self.max_cnt)))
        if k > 1:
            p = p[::k]
        d = max(self.min_dist, 1.0)
        key = (p[:, 0] // d).astype(np.int64) * 100000 + (p[:, 1] // d).astype(np.int64)
        _, first = np.unique(key, return_index=True)   # 每个 d×d 单元只留 1 个
        f = np.sort(first)
        if len(f) > self.max_cnt:
            idx = np.linspace(0, len(f) - 1, self.max_cnt).round().astype(np.int64)
            f = f[idx]
        return p[f]

    def _run_nms(self, thr):
        """把「降序 → MIN_DIST 欧氏抑制 → 前 max_cnt」整段交给 `nms_accel_0`。

        输入就是 ST 刚写好的响应图 buffer（**同一块内存，无需再搬**），
        输出 out_xy[i]=(y<<16)|x 与 out_n[0]=点数|(溢出<<31)。
        板上实测 60 点全流程 ~1.5 ms，而 Python 版要 7.8–11 ms。
        """
        t0 = time.perf_counter()
        _wr64(self.nm, N_RESP, self.rbuf.device_address)
        self.nm.write(N_ROWS, self.H)
        self.nm.write(N_COLS, self.W)
        self.nm.write(N_TH, int(thr))
        self.nm.write(N_MD, int(self.min_dist))
        self.nm.write(N_BR, PMAX)
        self.nm.write(N_MAXN, int(self.max_cnt))
        self.nm.write(N_AP, 1)
        c = 0
        while not ((self.nm.read(N_AP) >> 1) & 1):
            c += 1
            if c > 200000000:
                raise RuntimeError('NMS 超时')
        self.nbuf.sync_from_device()
        word = int(self.nbuf[-1])
        n = word & 0x7FFFFFFF
        ovf = (word >> 31) & 1
        if n:
            xy = np.asarray(self.nbuf[:n], dtype=np.uint32)
            pts = np.stack([(xy & 0xFFFF).astype(np.float64),
                            ((xy >> 16) & 0xFFFF).astype(np.float64)], 1)
        else:
            pts = np.zeros((0, 2))
        return pts, n, ovf, (time.perf_counter() - t0) * 1e3

    def _run_lk(self, l, pts_l, flow_l, n):
        if self.sm is None:
            raise RuntimeError(
                f'该位流没有 slk_accel_0（有：{self.ips}）—— pl/hybrid/st 前端需要它；'
                f'stcv2 不需要')

        """在第 l 层跑一次稀疏 LK。返回 (该层坐标下的绝对流, tracked)。

        ★★ 这个核的位移上限就是 **±5 px**（不是 ±8）：patch 恒以角点 (x0,y0) 为中心，
        `flow_io` 只是迭代起点，采样下标 `cj = dx + u + PMAX` 一旦 |u|>5 就越出
        21×21 patch（一维越到下一行，读到的是垃圾）⇒ 初值再准也会立即发散。
        输出端 `tofix` 又把结果钳在 ±8 px。
        ⇒ 金字塔只是把「每层的绝对位移」压到 ≤5 px；最细一层的位移必须 ≤5 px 才能
          精确收敛，所以本 IP 真正能精确跟踪的帧间位移约为 5 px。
        试过「按估计整像素重定心」的另一种接法：位移大时确实好转（20+px 桶 27→4.6 px），
        但 patch 内容被整体平移 = 跟错点（小位移桶 0.07→1.07 px），端到端更差，已否决。
        """
        hh, ww, nw = self.dims[l]
        # ★ 每次调用都重写全部地址/标量寄存器：共享 Overlay 时别的实例可能改过它们，
        #   而且「每层必须把 prev/cur 指到该层偏移」漏了就整层 tracked=0（见文件头坑 1）
        _wr64(self.sm, S_PREV, self.pb.device_address + 4 * self.off[l])
        _wr64(self.sm, S_CUR, self.cb.device_address + 4 * self.off[l])
        _wr64(self.sm, S_COR, self.a_cor)
        _wr64(self.sm, S_FLOW, self.a_flw)
        _wr64(self.sm, S_ST, self.a_sta)
        self.sm.write(S_ROWS, hh)
        self.sm.write(S_COLS, ww)

        cap = self.cap
        self.cfs[0:n] = (pts_l[:, 1].astype(np.uint32) << 16) | \
                        (pts_l[:, 0].astype(np.uint32) & 0xFFFF)
        self.cfs[cap:cap + n] = ((np.round(flow_l[:, 0] * (1 << self.q)).astype(np.int64) & 0xFFFF) << 16) | \
                                (np.round(flow_l[:, 1] * (1 << self.q)).astype(np.int64) & 0xFFFF)
        self.cfs[2 * cap:2 * cap + cap] = 0
        self.cfs.sync_to_device()

        self.sm.write(S_N, n)
        self.sm.write(S_ITER, self.max_iter)
        self.sm.write(S_AP, 1)
        c = 0
        while not ((self.sm.read(S_AP) >> 1) & 1):
            c += 1
            if c > 200000000:
                raise RuntimeError('LK 超时')
        self.cfs.sync_from_device()
        raw = np.asarray(self.cfs[cap:cap + n]).view(np.int16).reshape(-1, 2)
        st = np.asarray(self.cfs[2 * cap:2 * cap + n])
        u = raw[:, 1].astype(np.float64) / (1 << self.q)
        v = raw[:, 0].astype(np.float64) / (1 << self.q)
        return np.stack([u, v], 1), (st & 1).astype(bool)

    # ---------------------------------------------------------------- 只检点
    def _detect_hw(self, det='fast', th=None):
        """在**已经上传好的** self.pb 上检点，返回 (pts(N,2) float64, n_all, 该段耗时ms)。

        det='fast'：PL FAST 二值角点图 → 边界过滤/网格去重/抽稀（无响应值，不能按强度排）
        det='st'  ：PL Shi-Tomasi λ_min 响应图 → **cv2 等价后处理**（排序 → MIN_DIST 欧氏
                    抑制 → 前 N），这正是 LVI-SAM 原版 `goodFeaturesToTrack` 的语义。
                    ★ 阈值语义：cv2 的 qualityLevel=0.01 是「0.01 × 该帧最大 λ_min」，
                      不是绝对阈值 ⇒ 这里默认也按当帧 max 自适应（st_th=0 时），
                      **只跑一次 IP**（先按 0 阈值出全图，再在 CPU 上卡阈值 —— 比跑两遍 PL 便宜）。
        """
        t0 = time.perf_counter()
        if det == 'st':
            if self.tm is None:
                raise RuntimeError(f'该位流没有 shitomasi_accel_0（有：{self.ips}）')
            _wr64(self.tm, T_IN, self.pb.device_address)
            _wr64(self.tm, T_OUT, self.rbuf.device_address)
            self.tm.write(T_ROWS, self.H)
            self.tm.write(T_COLS, self.W)
            self.tm.write(T_K, 0)        # k=0 ⇒ 纯 Shi-Tomasi min(λ1,λ2)（= cv2 cornerMinEigenVal）
            self.tm.write(T_TH, int(th) if th else 0)
            self.tm.write(T_AP, 1)
            c = 0
            while not ((self.tm.read(T_AP) >> 1) & 1):
                c += 1
                if c > 200000000:
                    raise RuntimeError('Shi-Tomasi 超时')
            self.rbuf.sync_from_device()
            dt = (time.perf_counter() - t0) * 1e3

            t1 = time.perf_counter()
            # ---------- 硬件选点（nms_accel_0 在位流里时） ----------
            if self.nm is not None and self.nms in ('auto', 'on'):
                resp_v = np.asarray(self.rbuf).view(np.uint8).reshape(self.H, self.W)
                mv = int(resp_v[PMAX:self.H - PMAX, PMAX:self.W - PMAX].max())
                thr = int(th) if (th and int(th)) else max(1, int(round(0.01 * mv)))
                pts, n_cand, ovf, t_nms = self._run_nms(thr)
                for _ in range(3):          # 候选溢出（>4096）⇒ 抬阈值重跑
                    if not ovf:
                        break
                    thr = max(1, thr * 2)
                    pts, n_cand, ovf, dt2 = self._run_nms(thr)   # ★ 别用 dt：那是 ST 的耗时
                    t_nms += dt2
                self._n_cand, self._thr_used = int(n_cand), int(thr)
                self.t['st'] = dt
                self.t['nms'] = t_nms
                self.t['select'] = (time.perf_counter() - t1) * 1e3
                return pts, int(n_cand), dt
            # ---------- Python 选点（回退路径） ----------
            resp = np.array(self.rbuf).view(np.uint8).reshape(self.H, self.W)   # 必须复制
            # 边界安全区：LK 每层都要 ±PMAX 余量，越界点先抹掉（省掉后面一次布尔索引）
            resp[:PMAX, :] = 0
            resp[self.H - PMAX:, :] = 0
            resp[:, :PMAX] = 0
            resp[:, self.W - PMAX:] = 0
            thr = int(th) if (th and int(th)) else 0
            if thr <= 0:
                # cv2 qualityLevel=0.01 的等价物：相对当帧最大值
                mv = int(resp.max())
                thr = max(1, int(round(0.01 * mv)))
                if thr > 1:
                    resp[resp < thr] = 0
            if self.sel_mode == 'circle':
                pts, n_cand, n_drop = select_min_dist_circle(resp, self.max_cnt, self.min_dist,
                                                            self.sel_k, self.H, self.W)
            elif self.sel_k > 0:
                pts, n_cand, n_drop = select_min_dist_k(resp, self.max_cnt, self.min_dist,
                                                       self.sel_k)
            elif self.sel_fast:
                pts, n_cand, n_drop = select_min_dist_fast(resp, self.max_cnt, self.min_dist,
                                                           self.H, self.W)
            else:
                pts, n_cand, n_drop = select_min_dist(resp, self.max_cnt,
                                                     self.min_dist, self.max_cand)
            self._n_cand, self._thr_used = n_cand, int(thr)
            n_all = n_cand
            pts = pts.astype(np.float64)
            self.t['st'] = dt
            self.t['select'] = (time.perf_counter() - t1) * 1e3
            return pts, n_all, dt
        # ---- FAST ----
        if self.fm is None:
            raise RuntimeError(f'该位流没有 fast_accel_0（有：{self.ips}）')
        _wr64(self.fm, F_IN, self.pb.device_address)
        _wr64(self.fm, F_OUT, self.fout.device_address)
        self.fm.write(F_TH, self.th)
        self.fm.write(F_R, self.H)
        self.fm.write(F_C, self.W)
        self.fm.write(F_AP, 1)
        c = 0
        while not ((self.fm.read(F_AP) >> 1) & 1):
            c += 1
            if c > 200000000:
                raise RuntimeError('FAST 超时')
        self.fout.sync_from_device()
        dt = (time.perf_counter() - t0) * 1e3
        t1 = time.perf_counter()
        pl_all = self._fast_points()
        pts = self._select(pl_all)
        self.t['fast'] = dt
        self.t['select'] = (time.perf_counter() - t1) * 1e3
        return pts, len(pl_all), dt

    def detect(self, gray, det='fast', th=None):
        """**只检点**（上传 + PL 检点 + CPU 后处理），不做 LK。

        为什么单独暴露：板上实测 cv2 分支的 47 ms 里 `goodFeaturesToTrack` 独占 **45.9 ms**，
        `calcOpticalFlowPyrLK` 只 **6.1 ms** ⇒ 真正值得搬到 PL 的是「检点」，
        跟踪交给 CPU 的 3 层金字塔 LK 反而又快又准（本文件的 PL LK 大位移要靠金字塔兜）。
        `--frontend hybrid` 就是这条路：匹配 1.8+6.1 ≈ 8 ms，精度与 cv2 完全同级。
        """
        T = self.t
        for k in ('upload', 'fast', 'st', 'select', 'total'):
            T[k] = 0.0
        t_all = time.perf_counter()
        t0 = time.perf_counter()
        nw0 = self.dims[0][2]
        self.pb[0:nw0] = np.ascontiguousarray(gray).view(np.uint32).reshape(-1)
        self.pb.sync_to_device()
        T['upload'] = (time.perf_counter() - t0) * 1e3
        pts, n_all, _ = self._detect_hw(det, th)
        T['total'] = (time.perf_counter() - t_all) * 1e3
        return pts, n_all, dict(T)

    # ---------------------------------------------------------------- 主入口
    def match(self, prev_gray, cur_gray, det='fast', th=None):
        """prev_gray / cur_gray: (480,640) uint8 → 相邻两帧的匹配。"""
        T = self.t
        for k in ('pyramid', 'upload', 'fast', 'st', 'points', 'select', 'lk', 'total'):
            T[k] = 0.0
        t_all = time.perf_counter()
        L = self.levels

        # ① 金字塔
        t0 = time.perf_counter()
        import cv2
        pp, cc = [prev_gray], [cur_gray]
        for _ in range(L - 1):
            pp.append(cv2.pyrDown(pp[-1]))
            cc.append(cv2.pyrDown(cc[-1]))
        T['pyramid'] = (time.perf_counter() - t0) * 1e3

        # ② 上传两帧金字塔（各一次 sync）
        t0 = time.perf_counter()
        self._upload(self.pb, pp)
        self._upload(self.cb, cc)
        T['upload'] = (time.perf_counter() - t0) * 1e3

        # ③ 检点（prev 全分辨率；fast=FAST 二值 / st=Shi-Tomasi 响应图）
        pts, pl_all, _ = self._detect_hw(det, th)
        # ★ st 分支返回的是候选**计数**（int），fast 分支返回角点**数组** —— 这里统一成计数
        n_all = int(pl_all) if isinstance(pl_all, (int, np.integer)) else len(pl_all)
        T['points'] = n_all
        n = len(pts)
        if n < 8:
            T['total'] = (time.perf_counter() - t_all) * 1e3
            return dict(uv_prev=np.zeros((0, 2)), uv_cur=np.zeros((0, 2)),
                        ok=np.zeros(0, bool), n_fast=n_all, n_cand=n, t=dict(T))

        # ④ 由粗到细
        t0 = time.perf_counter()
        flow = np.zeros((n, 2))
        ok = np.zeros(n, bool)
        for l in range(L - 1, -1, -1):
            s = float(1 << l)
            hh, ww, _ = self.dims[l]
            pl = np.round(pts / s)
            m = ((pl[:, 0] >= PMAX) & (pl[:, 1] >= PMAX) &
                 (pl[:, 0] < ww - PMAX) & (pl[:, 1] < hh - PMAX))
            idx = np.nonzero(m)[0]
            if len(idx) == 0:
                continue
            out, tr = self._run_lk(l, pl[idx].astype(np.uint32), flow[idx] / s, len(idx))
            flow[idx[tr]] = out[tr] * s      # 细化结果 ×2^l 回全分辨率
            # 本层没跟上的点不在这一层判死（边界外/尺度下太弱），留到更细的层再试
            if l == 0:
                ok[idx] = tr                 # ★ 只在最细一层定生死
        T['lk'] = (time.perf_counter() - t0) * 1e3
        T['total'] = (time.perf_counter() - t_all) * 1e3
        self.n_call += 1
        return dict(uv_prev=pts[ok], uv_cur=(pts + flow)[ok], ok=ok,
                    n_fast=n_all, n_cand=n, t=dict(T))


# ------------------------------------------------------------------ 便捷入口
_M = None


def pl_match(prev_gray, cur_gray, **kw):
    """模块级便捷入口：首次调用建硬件，之后复用。"""
    global _M
    if _M is None:
        _M = PLMatcher(**kw)
    return _M.match(prev_gray, cur_gray)
