#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐段拆解 select 的耗时：到底花在哪（numpy 调用？纯 Python 循环？掩码写？）"""
import os
import sys
import time
import glob
from math import isqrt

import numpy as np
import cv2

H, W, PMAX = 480, 640, 10


def log(*a):
    print(*a)
    sys.stdout.flush()


for d in ["/home/ubuntu", "/home/ubuntu/merge_st"]:
    for f in ["_current_metadata.pkl", "global_pl_state.json"]:
        p = os.path.join(d, f)
        if os.path.exists(p):
            os.remove(p)
from pynq import Overlay, allocate  # noqa: E402

ol = Overlay("/home/ubuntu/merge_st/kv260_merged_st.bit")
st = ol.shitomasi_accel_0
rm, mm = st.register_map, st.mmio
inp = allocate(shape=(H, W), dtype=np.uint8)
out = allocate(shape=(H, W), dtype=np.uint8)
rm.img_in_1 = int(inp.device_address) & 0xFFFFFFFF
rm.img_in_2 = (int(inp.device_address) >> 32) & 0xFFFFFFFF
rm.img_out_1 = int(out.device_address) & 0xFFFFFFFF
rm.img_out_2 = (int(out.device_address) >> 32) & 0xFFFFFFFF
rm.rows, rm.cols, rm.k, rm.threshold = H, W, 0, 20

path = sorted(glob.glob("/home/ubuntu/kitti_cam/image_02/data/*.png"))[300]
img = cv2.imread(path, cv2.IMREAD_COLOR)
h2 = min(H, max(1, int(round(img.shape[0] * W / float(img.shape[1])))))
small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
canvas = np.zeros((H, W, 3), img.dtype)
top = (H - h2) // 2
canvas[top:top + h2] = small
gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
np.copyto(np.asarray(inp), gray)
inp.sync_to_device()
mm.write(0x00, 1)
while not ((mm.read(0x00) >> 1) & 1):
    pass
out.sync_from_device()
resp = np.array(out)
resp[:PMAX, :] = 0
resp[H - PMAX:, :] = 0
resp[:, :PMAX] = 0
resp[:, W - PMAX:] = 0
n_cand = int((resp > 0).sum())
log("候选 %d；python %s；loadavg %s" % (n_cand, sys.version.split()[0],
                                        open("/proc/loadavg").read().split()[0]))


def t(name, fn, n=7):
    fn()
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        r = fn()
        ts.append((time.perf_counter() - t0) * 1e3)
    log("  %-34s med=%7.3f ms" % (name, float(np.median(ts))))
    return r


ys0, xs0 = np.nonzero(resp)
o0 = np.argsort(-resp[ys0, xs0].astype(np.int32), kind="stable")
X0 = xs0[o0].tolist()
Y0 = ys0[o0].tolist()

t("np.nonzero(307k)", lambda: np.nonzero(resp))
t("resp[ys,xs] 花式索引", lambda: resp[ys0, xs0])
t("argsort(%d)" % len(xs0), lambda: np.argsort(-resp[ys0, xs0].astype(np.int32), kind="stable"))
t("xs[order].tolist()", lambda: xs0[o0].tolist())


def loop_empty():
    s = 0
    for i in range(n_cand):
        s += X0[i] + Y0[i]
    return s


def loop_ba_read():
    m = bytearray(H * W)
    s = 0
    for i in range(n_cand):
        s += m[Y0[i] * W + X0[i]]
    return s


def loop_list():
    pts = []
    for i in range(n_cand):
        pts.append((X0[i], Y0[i]))
    return pts


t("纯 Python 空循环 %d 次" % n_cand, loop_empty)
t("循环 + bytearray 读", loop_ba_read)
t("循环 + list.append", loop_list)


def mark100():
    m = bytearray(H * W)
    ONES = b"\x01" * (2 * 20 + 1)
    spans = [(dy, isqrt(20 * 20 - dy * dy - 1)) for dy in range(-20, 21)]
    for k in range(100):
        x = X0[k]
        y = Y0[k]
        for dy, dxm in spans:
            yy = y + dy
            if 0 <= yy < H:
                o = yy * W + x - dxm
                m[o:o + 2 * dxm + 1] = ONES[:2 * dxm + 1]
    return m


t("100 个点×41 行圆盘标记", mark100)


def rect100():
    m = np.zeros((H, W), np.uint8)
    rect = cv2.rectangle
    for k in range(100):
        x = X0[k]
        y = Y0[k]
        rect(m, (x - 20, y - 20), (x + 20, y + 20), 1, -1)
    return m


t("100 次 cv2.rectangle", rect100)


def nz100():
    m = np.zeros((H, W), np.uint8)
    s = 0
    for k in range(100):
        s += int(m[Y0[k], X0[k]])
    return s


t("100 次 m[y,x] numpy 标量读", nz100)

log("结论区（手工填）：看上面哪一行吃掉大头")
log("=== DONE ===")
