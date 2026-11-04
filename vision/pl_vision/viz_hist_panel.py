#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在光流叠图下面追加 |u| 分布直方图面板（数据来自 viz_hist.py 实测）"""
import numpy as np
import cv2

PNG = '/home/ubuntu/pl_vision/viz/flow_overlay.png'
BINS = ['0-2', '2-4', '4-6', '6-7', '7-8', '8-9', '>9']
SER = [('55->56  (turn starts)', [41.8, 23.5, 10.2, 2.0, 8.2, 14.3, 0.0], (117, 158, 29)),
       ('95->96  (mild)',        [32.0, 24.0,  7.0, 8.0, 20.0, 9.0, 0.0], (165, 95, 24)),
       ('127->128 (sharp turn)', [12.0, 11.0,  9.0, 8.0, 29.0, 31.0, 0.0], (23, 117, 186))]

PW, PH = 640, 250
panel = np.zeros((PH, PW, 3), np.uint8)
cv2.putText(panel, '|u| distribution of tracked points (% of valid points)',
            (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA)

X0, X1, Y0, Y1, YMAX = 78, 622, 44, 196, 45.0
gw = (X1 - X0) / float(len(BINS))
# 网格与 y 轴刻度
for v in (0, 10, 20, 30, 40):
    y = int(Y1 - v / YMAX * (Y1 - Y0))
    cv2.line(panel, (X0, y), (X1, y), (70, 70, 70), 1)
    cv2.putText(panel, '%d%%' % v, (36, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (170, 170, 170), 1, cv2.LINE_AA)
cv2.line(panel, (X0, Y1), (X1, Y1), (140, 140, 140), 1)
# 搜索窗边缘带（7~9 px 两格）
cv2.rectangle(panel, (int(X0 + 4 * gw), Y0), (int(X0 + 6 * gw), Y1), (48, 42, 40), -1)
cv2.putText(panel, 'search-window edge', (int(X0 + 4 * gw) + 6, Y0 - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (150, 130, 90), 1, cv2.LINE_AA)

bw, gap = 16, 3
for gi, lab in enumerate(BINS):
    gx = X0 + gi * gw
    for si, (name, vals, col) in enumerate(SER):
        h = int(vals[gi] / YMAX * (Y1 - Y0))
        bx = int(gx + gw / 2 - (3 * bw + 2 * gap) / 2 + si * (bw + gap))
        if h > 0:
            cv2.rectangle(panel, (bx, Y1 - h), (bx + bw, Y1), col, -1)
    cv2.putText(panel, lab, (int(gx + gw / 2 - 12), Y1 + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)
cv2.putText(panel, '|u| (px)', (int((X0 + X1) / 2) - 22, Y1 + 38),
            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1, cv2.LINE_AA)
# 图例
for si, (name, vals, col) in enumerate(SER):
    y = 216 + (si // 2) * 0
    x = 12 + si * 208
    cv2.rectangle(panel, (x, 202), (x + 14, 214), col, -1)
    cv2.putText(panel, name, (x + 20, 213), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (215, 215, 215), 1, cv2.LINE_AA)
cv2.putText(panel, 'no point exceeds 9 px: the LK search window (WH=5/SEARCH=4) is the hard ceiling',
            (10, 242), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (140, 180, 235), 1, cv2.LINE_AA)

img = cv2.imread(PNG)
out = np.vstack([img, panel])
cv2.imwrite('/home/ubuntu/pl_vision/viz/flow_and_hist.png', out)
print('WROTE', out.shape)
