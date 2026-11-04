#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_img2.py —— 读图四条路线对比（决定「预缩放」是离线红利还是能在线拿到）。"""
import sys, time
import numpy as np, cv2
SRC='/home/ubuntu/kitti_cam/image_02/data'; PRE='/home/ubuntu/kitti_pre'
W,H=640,480
FR=[0,40,80,120,160,200]

def pad(r):
    out=np.zeros((H,W),np.uint8); nh=r.shape[0]; top=(H-nh)//2; out[top:top+nh]=r; return out

def a_color(f):            # 现状：彩色解码 → INTER_AREA 到 640 宽 → pad → 灰度
    im=cv2.imread(f'{SRC}/{f:010d}.png'); nh=int(round(im.shape[0]*W/im.shape[1]))
    r=cv2.resize(im,(W,nh),interpolation=cv2.INTER_AREA)
    o=np.zeros((H,W,3),np.uint8); top=(H-nh)//2; o[top:top+nh]=r
    return cv2.cvtColor(o,cv2.COLOR_BGR2GRAY)
def b_gray(f):             # 在线：灰度解码 → resize → pad（省掉 cvtColor 与 3 通道缩放）
    im=cv2.imread(f'{SRC}/{f:010d}.png',cv2.IMREAD_GRAYSCALE); nh=int(round(im.shape[0]*W/im.shape[1]))
    return pad(cv2.resize(im,(W,nh),interpolation=cv2.INTER_AREA))
def c_red2(f):             # 在线：半分辨率解码（613×185）→ 缩到 640 宽 → pad
    im=cv2.imread(f'{SRC}/{f:010d}.png',cv2.IMREAD_REDUCED_GRAYSCALE_2)
    nh=int(round(im.shape[0]*W/im.shape[1])); return pad(cv2.resize(im,(W,nh),interpolation=cv2.INTER_AREA))
def d_pre(f):              # 离线预缩放：直读 640×480 灰度
    return cv2.imread(f'{PRE}/{f:010d}.png',cv2.IMREAD_GRAYSCALE)

ref={f:a_color(f) for f in FR}
for tag,fn in (('A 彩色解码+resize+cvtColor（现状）',a_color),
               ('B 灰度解码+resize           ',b_gray),
               ('C 半分辨率解码+resize       ',c_red2),
               ('D 预缩放灰度直读（离线）    ',d_pre)):
    ts=[]
    for f in FR:
        t0=time.perf_counter(); g=fn(f); ts.append((time.perf_counter()-t0)*1e3)
    d=np.abs(g.astype(int)-ref[FR[-1]].astype(int)).mean() if g is not None and g.shape==ref[FR[-1]].shape else -1
    print(f'  {tag}: {np.median(ts):6.1f} ms/张   与 A 的平均像素差 {d:.2f}')
