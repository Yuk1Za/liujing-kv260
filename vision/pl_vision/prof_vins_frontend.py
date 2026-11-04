#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VINS-Mono / LVI-SAM 视觉前端在 KV260 上的耗时分解
= 决定「哪一段值得搬去 PL」的依据。
CLAHE + goodFeaturesToTrack(Shi-Tomasi) + calcOpticalFlowPyrLK(21x21,3层) + rejectWithF(RANSAC) + setMask
"""
import os, time
import numpy as np, cv2

IMGDIR = '/home/ubuntu/kitti_cam/image_02/data'
W, H = 640, 480
MAX_CNT, QLVL, MIN_DIST = 150, 0.01, 30
REP = 30


def grab(i):
    img = cv2.imread(os.path.join(IMGDIR, '%010d.png' % i), cv2.IMREAD_COLOR)
    h0, w0 = img.shape[:2]
    h2 = min(H, max(1, int(round(h0 * W / float(w0)))))
    small = cv2.resize(img, (W, h2), interpolation=cv2.INTER_AREA)
    cv = np.zeros((H, W, 3), np.uint8); top = (H - h2) // 2
    cv[top:top + h2] = small
    return cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)


def med(fn, rep=REP):
    fn(); ts = []
    for _ in range(rep):
        t0 = time.perf_counter(); fn(); ts.append((time.perf_counter() - t0) * 1e3)
    return float(np.median(ts))


prev = grab(95); cur = grab(96)
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
p1 = clahe.apply(prev); p2 = clahe.apply(cur)

for nthr in (1, 4):
    cv2.setNumThreads(nthr)
    print('\n===== cv2 线程数 = %d =====' % nthr)
    t_clahe = med(lambda: clahe.apply(prev))
    m = np.ones((H, W), np.uint8) * 255
    t_gftt = med(lambda: cv2.goodFeaturesToTrack(p2, MAX_CNT, QLVL, MIN_DIST, mask=m))
    pts = cv2.goodFeaturesToTrack(p2, MAX_CNT, QLVL, MIN_DIST, mask=m)
    t_klt = med(lambda: cv2.calcOpticalFlowPyrLK(p2, p1, pts, None, winSize=(21, 21), maxLevel=2,
                                                 criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 10, 0.05)))
    _, st, _ = cv2.calcOpticalFlowPyrLK(p2, p1, pts, None, winSize=(21, 21), maxLevel=2,
                                        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 10, 0.05))
    good_p = pts[st.ravel() == 1]; good_q, _s2, _e2 = cv2.calcOpticalFlowPyrLK(p2, p1, good_p, None, winSize=(21, 21), maxLevel=2)
    t_f = med(lambda: cv2.findFundamentalMat(good_p, good_q, cv2.FM_RANSAC, 1.0, 0.99))

    def mask_step():
        mm = np.ones((H, W), np.uint8) * 255
        for p in good_p[:MAX_CNT]:
            cv2.circle(mm, (int(p[0][0]), int(p[0][1])), MIN_DIST, 0, -1)
    t_mask = med(mask_step)

    tot = t_clahe + t_gftt + t_klt + t_f + t_mask
    print('  CLAHE               %7.2f ms' % t_clahe)
    print('  goodFeaturesToTrack %7.2f ms   (Shi-Tomasi, %d 点)' % (t_gftt, len(pts)))
    print('  calcOpticalFlowPyrLK%7.2f ms   (21x21, 3 层)' % t_klt)
    print('  rejectWithF(RANSAC) %7.2f ms' % t_f)
    print('  setMask             %7.2f ms' % t_mask)
    print('  ----------------------------------')
    print('  前端合计            %7.2f ms   (= %.0f FPS 上限)' % (tot, 1000.0 / tot))
cv2.setNumThreads(4)
