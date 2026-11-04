#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chk_st.py —— 校验 PL Shi-Tomasi 检点与 cv2.goodFeaturesToTrack 的等价性 + 计时。

LVI-SAM 原版前端是 `goodFeaturesToTrack(img, MAX_CNT, 0.01, MIN_DIST, mask)`，
本脚本回答三个问题：
  1) PL ST 响应图给出的点集与 cv2 重合多少（2 px 容差、双向匹配）？
  2) 自适应阈值（0.01×max）合不合适（候选数 n_cand 会不会爆）？
  3) 检点 + PL LK 一整段多少 ms，PL LK 的流与 cv2 LK 差多少？

用法（板端，需 root）：
  sudo -n -E env PL_BIT=/home/ubuntu/vis3/design_1_wrapper_stlk4.bit PL_Q=4 python3 chk_st.py
"""
import os
import sys
import time

import numpy as np
import cv2

sys.path.insert(0, '/home/ubuntu/lvi_board')
from pl_match import PLMatcher, BIT  # noqa: E402
# ★ 必须复用节点的预处理（缩放到 640×193 再 letterbox 到 480×640）：
#   直接读原始 1226×370 的图 (a) 宽度不是 4 的倍数，uint32 视图直接报错；
#   (b) 与节点看到的图不同，点集/时序都不能代表实际运行。
from vo_pnp_node import grab_img  # noqa: E402

H = os.environ.get('LVI_HOME', '/home/ubuntu')
IMGDIR = f'{H}/kitti_cam/image_02/data'
NIMG = int(os.environ.get('NIMG', '6'))
MAXCNT = int(os.environ.get('MAXCNT', '60'))
MIND = int(os.environ.get('MIND', '20'))
TH = int(os.environ.get('ST_TH', '0'))
MC = int(os.environ.get('MAXCAND', '0'))
LEVELS = int(os.environ.get('PL_LEVELS', '3'))


def log(*a):
    print(*a)
    sys.stdout.flush()


def rep(a, b, tol=2.0):
    """双向最近邻匹配率：a 里有多少点能在 b 里找到 tol 内的点（反之亦然）"""
    if len(a) == 0 or len(b) == 0:
        return 0.0, 0.0
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    return float((d.min(1) <= tol).mean()), float((d.min(0) <= tol).mean())


def main():
    m = PLMatcher(bit=BIT, max_cnt=MAXCNT, min_dist=MIND, levels=LEVELS,
                  st_th=TH, max_cand=MC)
    log(f'PL IP: {m.ips}')
    log(f'阈值 st_th={TH}（0=自适应 0.01×max）  max_cand={MC}  MIN_DIST={MIND}  max_cnt={MAXCNT}')

    imgs = [cv2.cvtColor(grab_img(i, 640, 480), cv2.COLOR_BGR2GRAY) for i in range(NIMG)]
    assert all(x is not None for x in imgs), '读图失败'
    log(f'图 {NIMG} 张 {imgs[0].shape}')

    # ---------- 1) 检点一致性 ----------
    log('\n=== 检点：PL ST vs cv2.goodFeaturesToTrack(0.01, %d) ===' % MIND)
    fr_pl, fr_cv, t_pl, t_cv, nc, ths = [], [], [], [], [], []
    for k, g in enumerate(imgs):
        t0 = time.perf_counter()
        pts, n_all, tt = m.detect(g, det='st')
        t_pl.append((time.perf_counter() - t0) * 1e3)
        nc.append(n_all)
        ths.append(getattr(m, '_thr_used', 0))
        t0 = time.perf_counter()
        c0 = cv2.goodFeaturesToTrack(g, MAXCNT, 0.01, MIND)
        t_cv.append((time.perf_counter() - t0) * 1e3)
        c0 = np.zeros((0, 2)) if c0 is None else c0.reshape(-1, 2)
        a, b = rep(pts.astype(np.float64), c0.astype(np.float64))
        fr_pl.append(a)
        fr_cv.append(b)
        log(f'  #{k}: PL {len(pts):3d} 点（候选 {n_all}，阈值 {ths[-1]}） cv2 {len(c0):3d} 点  '
            f'PL→cv2 {a*100:.0f}%  cv2→PL {b*100:.0f}%  '
            f'耗时 PL {t_pl[-1]:.1f} ms（其中 select {tt["select"]:.1f}）/ cv2 {t_cv[-1]:.1f} ms')
    log(f'  合计：PL→cv2 {np.mean(fr_pl)*100:.1f}%  cv2→PL {np.mean(fr_cv)*100:.1f}%  '
        f'PL {np.median(t_pl):.1f} ms vs cv2 {np.median(t_cv):.1f} ms（中位）')

    # ---------- 2) 整段匹配：ST + PL LK ----------
    log('\n=== 匹配：ST + PL LK vs cv2(gftt+3层LK) ===')
    rows = []
    for k in range(NIMG - 1):
        g1, g2 = imgs[k], imgs[k + 1]
        r = m.match(g1, g2, det='st')
        up, uc = r['uv_prev'], r['uv_cur']
        p0 = cv2.goodFeaturesToTrack(g1, MAXCNT, 0.05, MIND)
        p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0, None, winSize=(21, 21), maxLevel=3)
        p0, p1 = p0.reshape(-1, 2)[st.reshape(-1).astype(bool)], \
                 p1.reshape(-1, 2)[st.reshape(-1).astype(bool)]
        # 只在「两边都检到同一批点」的子集上比流：把 PL 的点配到 cv2 点集里 1 px 内
        d = np.linalg.norm(up[:, None, :] - p0[None, :, :], axis=2)
        j = d.argmin(1)
        msk = d[np.arange(len(up)), j] <= 1.5
        err = np.linalg.norm(uc[msk] - p1[j[msk]], axis=1) if msk.any() else np.array([np.nan])
        rows.append((k, len(up), len(p0), msk.mean() * 100, np.median(err),
                     r['t']['st'], r['t']['select'], r['t']['lk'], r['t']['total']))
        log(f'  {k}->{k+1}: PL 跟踪 {len(up):3d} / cv2 {len(p0):3d}  配对 {msk.mean()*100:.0f}%  '
            f'流误差中位 {np.median(err):.3f} px | ST {r["t"]["st"]:.1f} + select {r["t"]["select"]:.1f}'
            f' + LK {r["t"]["lk"]:.1f} = {r["t"]["total"]:.1f} ms')
    if rows:
        A = np.array([r[4:] for r in rows], float)
        log(f'\n  中位：配对率 {np.median([r[3] for r in rows]):.0f}%  流误差 {np.nanmedian(A[:, 0]):.3f} px  '
            f'ST {np.median(A[:, 1]):.1f}  select {np.median(A[:, 2]):.1f}  '
            f'LK {np.median(A[:, 3]):.1f}  合计 {np.median(A[:, 4]):.1f} ms')


if __name__ == '__main__':
    main()
