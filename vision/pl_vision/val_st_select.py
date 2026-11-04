#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证 pl_vision_node.py 的 ST 后处理（响应图 -> 降序 -> MIN_DIST 方形抑制 -> 前 N）

从节点源码里用 AST 抽出**真实函数** select_min_dist 来测（不复制粘贴，避免验证副本走偏）。

两级对拍：
  L1  select_min_dist 作用在「cv2 浮点响应 ×1024」上  vs  cv2.goodFeaturesToTrack
      （同 qualityLevel、同 minDistance、同边界 mask）-> 验证 CPU 后处理与 OpenCV 同规则
  L2  PL 8bit 响应图 + select_min_dist              vs  cv2.goodFeaturesToTrack
      （按候选数配平 qualityLevel）-> 验证 IP 响应图语义端到端可用
  L3  计时：select_min_dist 在真实候选数下的耗时

尺度事实（本板实测）：PL 输出的就是原始 λ_min（0…255 饱和）；而 cv2.cornerMinEigenVal
是它除以 1024 —— 所以 L1 里把 cv 响应 ×1024 才与 PL 同单位。

用法（板端，需 root）：
  sudo /usr/local/share/pynq-venv/bin/python val_st_select.py
"""
import ast
import os
import sys
import glob
import time

import numpy as np
import cv2

NODE = "/home/ubuntu/pl_vision/pl_vision_node.py"
BIT = os.environ.get("BIT", "/home/ubuntu/merge_st/kv260_merged_st.bit")
IMGDIR = "/home/ubuntu/kitti_cam/image_02/data"
T = int(os.environ.get("T", "20"))
NC = int(os.environ.get("NC", "100"))
MIN_DIST = int(os.environ.get("MIN_DIST", "20"))
H, W = 480, 640
PMAX = 10
FRAMES = [int(x) for x in os.environ.get("FRAMES", "0,60,300").split(",")]
CVRATIO = 1024.0          # cv2.cornerMinEigenVal = PL λ_min / 1024（实测）


def log(*a):
    print(*a)
    sys.stdout.flush()


# ---------------- 从节点源码抽真实函数 ----------------
src = open(NODE, encoding="utf-8").read()
tree = ast.parse(src)
fns = [n for n in tree.body if isinstance(n, ast.FunctionDef)
       and n.name in ("select_min_dist", "_ensure_xclbin")]
assert len(fns) == 2, "从节点里没抽到两个函数：%s" % [f.name for f in fns]
g = {"np": np, "os": os, "shutil": __import__("shutil"), "cv2": cv2}
exec(compile(ast.Module(body=fns, type_ignores=[]), NODE, "exec"), g)
select_min_dist = g["select_min_dist"]
import hashlib  # noqa: E402
log("已从节点源码抽出 select_min_dist（源码 md5 %s）" % hashlib.md5(src.encode()).hexdigest()[:8])

# ---------------- 加载 PL ----------------
for d in ["/home/ubuntu", "/home/ubuntu/merge_st"]:
    for f in ["_current_metadata.pkl", "global_pl_state.json"]:
        p = os.path.join(d, f)
        if os.path.exists(p):
            os.remove(p)

from pynq import Overlay, allocate  # noqa: E402

g["_ensure_xclbin"](BIT, log)
ol = Overlay(BIT)
ips = sorted(ol.ip_dict)
log("PL IP: %s" % ips)
assert "shitomasi_accel_0" in ips, "该 bit 里没有 shitomasi_accel_0"
st = ol.shitomasi_accel_0
rm, mm = st.register_map, st.mmio
inp = allocate(shape=(H, W), dtype=np.uint8)
out = allocate(shape=(H, W), dtype=np.uint8)
rm.img_in_1 = int(inp.device_address) & 0xFFFFFFFF
rm.img_in_2 = (int(inp.device_address) >> 32) & 0xFFFFFFFF
rm.img_out_1 = int(out.device_address) & 0xFFFFFFFF
rm.img_out_2 = (int(out.device_address) >> 32) & 0xFFFFFFFF
rm.rows = H
rm.cols = W
rm.k = 0
rm.threshold = T


def st_run():
    mm.write(0x00, 1)
    t0 = time.perf_counter()
    while not ((mm.read(0x00) >> 1) & 1):
        if time.perf_counter() - t0 > 5.0:
            raise RuntimeError("ST 超时")
        time.sleep(0.0002)
    return time.perf_counter() - t0


def grab(path):
    """与节点 _grab 完全一致：等比缩放到宽 640 + 上下补零 + 转灰度"""
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    h0, w0 = img.shape[:2]
    if (w0, h0) != (W, H):
        h2 = min(H, max(1, int(round(h0 * W / float(w0)))))
        small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
        canvas = np.zeros((H, W, img.shape[2]), img.dtype)
        top = (H - h2) // 2
        canvas[top:top + h2] = small
        img = canvas
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def border_zero(a):
    a[:PMAX, :] = 0
    a[H - PMAX:, :] = 0
    a[:, :PMAX] = 0
    a[:, W - PMAX:] = 0
    return a


MASK_INT = np.zeros((H, W), np.uint8)
MASK_INT[PMAX:H - PMAX, PMAX:W - PMAX] = 255


def f1_at(p_ref, p_tst, tol):
    """以 p_ref 为基准，半径 tol 内一对一贪婪匹配"""
    if len(p_ref) == 0 or len(p_tst) == 0:
        return 0.0, 0.0, 0.0, 0.0
    d = np.linalg.norm(p_ref[:, None, :].astype(np.float32)
                       - p_tst[None, :, :].astype(np.float32), axis=2)
    used = np.zeros(len(p_tst), bool)
    hit = 0
    for i in range(len(p_ref)):
        j = np.where(~used)[0]
        if len(j) == 0:
            break
        k = j[np.argmin(d[i, j])]
        if d[i, k] <= tol:
            hit += 1
            used[k] = True
    pr = hit / float(len(p_tst))
    rc = hit / float(len(p_ref))
    f1 = 0.0 if pr + rc == 0 else 2 * pr * rc / (pr + rc)
    return f1, pr, rc, float(np.mean(np.min(d, axis=1)))


def gftt(gray, maxc, q):
    c = cv2.goodFeaturesToTrack(gray, maxCorners=maxc, qualityLevel=q,
                                minDistance=MIN_DIST, mask=MASK_INT, blockSize=3,
                                useHarrisDetector=False)
    return np.zeros((0, 2), np.float32) if c is None else c.reshape(-1, 2)


files = sorted(glob.glob(os.path.join(IMGDIR, "*.png")))
log("图像 %d 帧；T=%d NC=%d MIN_DIST=%d CVRATIO=%.0f" % (len(files), T, NC, MIN_DIST, CVRATIO))
st_run()  # warmup
summary = []
for fidx in FRAMES:
    if fidx >= len(files):
        continue
    gray = grab(files[fidx])

    # ---- PL 响应图 ----
    np.copyto(np.asarray(inp), gray)
    inp.sync_to_device()
    t_ip = st_run()
    out.sync_from_device()
    resp_pl = border_zero(np.array(out))          # uint8
    t0 = time.perf_counter()
    pts_pl, n_cand, n_skip = select_min_dist(resp_pl, NC, MIN_DIST)
    t_sel = (time.perf_counter() - t0) * 1e3

    # ---- cv2 浮点响应（= PL / 1024）----
    resp_cv = cv2.cornerMinEigenVal(gray, blockSize=3, ksize=3)   # λ_min，float32
    mx_all = float(resp_cv.max())
    inner = (slice(PMAX, H - PMAX), slice(PMAX, W - PMAX))

    # 用候选数配平 qualityLevel
    lo, hi = 1e-7, 0.5
    for _ in range(60):
        mid = (lo * hi) ** 0.5
        cnt = int((resp_cv[inner] >= mid * mx_all).sum())
        if cnt > n_cand:
            lo = mid
        else:
            hi = mid
    q = (lo * hi) ** 0.5

    # L1：同一张 cv 响应（×1024 换成 PL 单位、round 成整数）-> 我的贪心 vs cv2 贪心
    resp_q = np.round(resp_cv * CVRATIO).astype(np.int32)
    border_zero(resp_q)
    thr_abs = int(round(q * mx_all * CVRATIO))
    resp_q[resp_q < max(thr_abs, 1)] = 0
    resp_q = np.clip(resp_q, 0, 255).astype(np.uint8)   # select_min_dist 要 uint8
    pts_l1, cand_l1, _ = select_min_dist(resp_q, NC, MIN_DIST)
    cv_all = gftt(gray, 10 ** 6, q)                 # cv2 自己做完抑制后的全部点
    cv_l1 = np.round(cv_all).astype(np.int32)[:NC]
    r_l1 = f1_at(np.round(cv_all).astype(np.int32), pts_l1, 1.0)
    # 只比「候选集」时（minDistance=0）两边各有多少
    cv_cand = gftt(gray, 10 ** 6, q) if MIN_DIST == 0 else None

    # L2：PL 响应图 vs cv2 GFTT —— 关键是**用同一个绝对阈值**
    #     PL 的 threshold=T 是 λ_min 原始值；cv 的 qualityLevel 是相对 max 的比例
    #     => 逼平：q_T = (T / 1024) / max。用 q（按候选数配平）会把阈值抬高 ~2×，
    #        两侧候选集分布完全不同，F1 会被误读成"IP 错"。
    q_T = (T / CVRATIO) / mx_all
    cv_l2 = np.round(gftt(gray, NC, q_T)).astype(np.int32)
    r_l2 = f1_at(cv_l2, pts_pl, 2.0)
    r_l2b = f1_at(cv_l2, pts_pl, 5.0)
    cnt_T = int((resp_cv[inner] >= T / CVRATIO).sum())

    # 尺度核对
    m = (resp_pl > 0)
    ratio = float(np.median(resp_cv[m] * CVRATIO / resp_pl[m]))
    rho = float(np.corrcoef(resp_pl[m].astype(float), resp_cv[m].astype(float))[0, 1])

    log("\n=== frame %d ===" % fidx)
    log("  PL: 候选 %d -> 取 %d（抑制掉 %d）  IP %.3f ms  选择 %.3f ms"
        % (n_cand, len(pts_pl), n_skip, t_ip * 1e3, t_sel))
    log("  配平 qualityLevel=%.6f（abs PL 阈值≈%d，该阈值下 cv 内部候选 %d）"
        % (q, thr_abs, cand_l1))
    log("  L1 我的贪心 vs cv2（同响应同阈值同 mask，容差 1px）：F1=%.4f P=%.4f R=%.4f 平均最近距 %.2f px"
        % r_l1)
    log("  L2 PL 响应图 vs cv2 GFTT【同绝对阈值 q_T=%.4f】容差 2px：F1=%.4f P=%.4f R=%.4f 平均最近距 %.2f px"
        % ((q_T,) + r_l2))
    log("     该阈值下：cv 候选 %d / PL 候选 %d" % (cnt_T, n_cand))
    log("     （容差 5px：F1=%.4f P=%.4f R=%.4f 平均最近距 %.2f px）" % r_l2b)
    log("  点数：PL %d / cv2(L2) %d / cv2(L1 全部) %d / L1 我的 %d"
        % (len(pts_pl), len(cv_l2), len(cv_all), len(pts_l1)))
    log("  尺度：cv×1024/PL 中位比 %.3f，rho=%.4f（共同支撑 %d 像素）"
        % (ratio, rho, int(m.sum())))
    summary.append((fidx, len(pts_pl), t_ip * 1e3, t_sel, r_l1[0], r_l2[0], r_l2b[0]))

log("\n==== 汇总 ====")
log("%6s %5s %8s %8s %8s %8s %9s" % ("frame", "N", "IP_ms", "sel_ms", "L1_F1", "L2_F1", "L2_F1@5"))
for r in summary:
    log("%6d %5d %8.3f %8.3f %8.4f %8.4f %9.4f" % r)

# 选点可视化
if summary:
    fidx = summary[0][0]
    gray = grab(files[fidx])
    np.copyto(np.asarray(inp), gray)
    inp.sync_to_device()
    st_run()
    out.sync_from_device()
    resp_pl = border_zero(np.array(out))
    pts_pl, _, _ = select_min_dist(resp_pl, NC, MIN_DIST)
    q = 0
    cvp = gftt(gray, NC, 0.01)
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for x, y in pts_pl:
        cv2.circle(vis, (int(x), int(y)), 4, (0, 0, 255), 1)
    for x, y in cvp:
        cv2.circle(vis, (int(x), int(y)), 2, (0, 255, 0), -1)
    p = "/home/ubuntu/merge_st/val_pts_%d.png" % fidx
    cv2.imwrite(p, vis)
    log("已存选点可视化 %s（红圈=PL+MIN_DIST，绿点=cv2 GFTT q=0.01）" % p)
log("=== DONE ===")
