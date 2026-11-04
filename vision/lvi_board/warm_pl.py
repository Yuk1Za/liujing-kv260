#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""warm_pl.py —— 预热 PYNQ 元数据缓存（把缓存写成"目标位流"的版本）。

为什么需要：共存时视觉节点用 PL_CLEAR_CACHE=0（不能清缓存，否则可能重新解析/下载位流）。
但如果缓存是**上一颗位流**留下的，Overlay() 解析出来的 ip_dict 就不对。
做法：部署新位流后，先单独跑一次本脚本（清缓存 → 加载 → 退出），
之后雷达链和视觉节点都走 PL_CLEAR_CACHE=0 直接复用这份缓存。

用法：sudo python3 warm_pl.py /home/ubuntu/vis3/design_1_wrapper_fusion.bit
"""
import os
import sys

PYNQ_SP = '/usr/local/share/pynq-venv/lib/python3.10/site-packages'


def main():
    bit = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/vis3/design_1_wrapper_fusion.bit'
    if PYNQ_SP not in sys.path:
        sys.path.insert(0, PYNQ_SP)
    psp = PYNQ_SP + '/pynq/pl_server'
    for f in (f'{psp}/_current_metadata.pkl', f'{psp}/global_pl_state.json'):
        if os.path.exists(f):
            os.remove(f)
            print(f'removed {f}')
    from pynq import Overlay
    o = Overlay(bit)
    ips = [k for k in o.ip_dict if 'accel' in k or 'feature' in k]
    print(f'warm ok: {bit}')
    print(f'  ip: {ips}')


if __name__ == '__main__':
    main()
