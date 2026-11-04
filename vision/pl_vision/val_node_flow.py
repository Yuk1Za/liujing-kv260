#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑「上板那份节点代码」，三档 lk_prior 在同一批 ST 角点上 A/B（cv2 3 层 LK 当参比）。

不复制逻辑：import 源码里的 PlVisionNode，monkey-patch _pub 截流。
"""
import importlib.util
import sys

import numpy as np
import cv2

spec = importlib.util.spec_from_file_location("plv", "/home/ubuntu/pl_vision/pl_vision_node.py")
plv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plv)

import rclpy  # noqa: E402

rclpy.init(args=['--ros-args',
                 '-p', 'bit:=/home/ubuntu/merge_st/kv260_merged_st.bit',
                 '-p', 'detector:=st',
                 '-p', 'threshold:=20',
                 '-p', 'min_dist:=20',
                 '-p', 'max_corners:=100'])
node = plv.PlVisionNode()

cap = {}
node._pub = lambda pub, arr, shape: cap.__setitem__(id(pub), arr)
files = node.files


def reset():
    node.prev = None
    node.track_pts = None
    node.field_pts = node.field_u = node.field_v = None


def sweep(mode, k0, n):
    """返回 [(k, pl_u, cv_u, ok)]；同时把每帧耗时记下来"""
    reset()
    node.lk_prior = mode
    out = []
    for k in range(k0 - 1, k0 + n):
        corners = node.track_pts.copy() if (node.prev is not None and node.track_pts is not None) else None
        cap.clear()
        node._process(k)
        flow = cap.get(id(node.pub_flow))
        if k < k0 or flow is None or corners is None or len(corners) == 0:
            continue
        prev, cur = node._grab(files[k - 1]), node._grab(files[k])
        r, s, _ = cv2.calcOpticalFlowPyrLK(
            prev, cur, corners.astype(np.float32).reshape(-1, 1, 2), None,
            winSize=(11, 11), maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        cu = (r.reshape(-1, 2) - corners.astype(np.float32))[:, 0]
        cok = s.reshape(-1) == 1
        ok = flow[:, 4] == 1
        out.append((k, float(np.median(flow[ok, 2])) if ok.sum() else np.nan,
                    float(np.median(cu[cok])) if cok.sum() else np.nan, int(ok.sum())))
        sys.stdout.flush()
    return out


def report(label, k0, n):
    print("\n=== %s (frame %d..%d) ===" % (label, k0, k0 + n - 1))
    res = {m: sweep(m, k0, n) for m in ('none', 'const', 'affine')}
    print(" %5s | %8s | %8s %8s %8s | %s" % ("k", "cv2", "none", "const", "affine", "胜者"))
    acc = {m: 0.0 for m in res}
    signbad = {m: 0 for m in res}
    cnt = 0
    for i in range(len(res['none'])):
        k = res['none'][i][0]
        c = res['none'][i][2]
        row = [res[m][i][1] for m in ('none', 'const', 'affine')]
        for m, v in zip(('none', 'const', 'affine'), row):
            if v == v:
                acc[m] += abs(v - c)
        if abs(c) > 0.3:
            for m, v in zip(('none', 'const', 'affine'), row):
                if v == v and np.sign(v) != np.sign(c):
                    signbad[m] += 1
        best = min((('none', abs(row[0] - c)), ('const', abs(row[1] - c)),
                    ('affine', abs(row[2] - c))), key=lambda t: t[1])[0]
        cnt += 1
        print(" %5d | %+8.2f | %+8.2f %+8.2f %+8.2f | %s" % (k, c, row[0], row[1], row[2], best))
    print("  ⇒ 平均 |Δ中位流|：none %.3f | const %.3f | affine %.3f px；符号错帧数 %d/%d %d/%d %d/%d"
          % (acc['none'] / cnt, acc['const'] / cnt, acc['affine'] / cnt,
             signbad['none'], cnt, signbad['const'], cnt, signbad['affine'], cnt))


report("缓转弯", 62, 16)
report("急转弯（cv2 中位流 2–9 px）", 293, 14)
report("直行", 700, 10)

node.destroy_node()
rclpy.shutdown()
