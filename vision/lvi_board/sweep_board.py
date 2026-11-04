#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""板上参数扫描：点云抽稀 × 特征数，看「step 耗时 / 赋深率 / 精度」三者的权衡。

跑在板上（路径为板上布局）。每个配置独立进程，避免 numpy/cv2 状态串味。
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ENV = dict(os.environ)
ENV.update(
    VO_IMG=f'{os.path.expanduser("~")}/kitti_cam/image_02/data',
    VO_VEL=f'{os.path.expanduser("~")}/_kitti_incoming/velodyne_points/data',
    VO_CAL=f'{os.path.expanduser("~")}/kiss_icp_fpga/evidence/reference_raw_v3',
    VO_GT=f'{HERE}/gt_pose_fixed.csv',
)

CFGS = [(1, 150), (2, 150), (3, 150), (1, 80), (2, 80), (3, 80)]
W = int(sys.argv[1]) if len(sys.argv) > 1 else 40

print(f'{"stride":>6} {"max_cnt":>7} | {"step":>7} {"赋深":>7} {"投影":>7} {"PnP":>6} {"cov":>6} | '
      f'{"位置RMS":>8} {"旋转RMS":>7}')
print('-' * 78)
rows = []
for st, mc in CFGS:
    out = f'/tmp/s{st}_{mc}.json'
    r = subprocess.run([sys.executable, f'{HERE}/bench_vo.py', '--F0', '0', '--W', str(W),
                        '--preload', '--stride', str(st), '--max-cnt', str(mc), '--out', out],
                       capture_output=True, text=True, env=ENV, cwd=HERE)
    if not os.path.exists(out):
        print(f'{st:>6} {mc:>7} | 失败: {r.stderr.strip().splitlines()[-1][:60] if r.stderr else "?"}')
        continue
    d = json.load(open(out))
    m = d['ms_per_frame']
    rows.append((st, mc, m, d['pos_rms'], d['rot_rms']))
    print(f'{st:>6} {mc:>7} | {m["step"]:7.1f} {m["depth"]:7.1f} {m["lidar"]:7.1f} '
          f'{m["pnp"]:6.1f} {m["cov"]:6.1f} | {d["pos_rms"]:8.3f} {d["rot_rms"]:7.2f}')
    os.remove(out)

if rows:
    # 以「step 最快且位置 RMS 不劣于基线 20%」为选优口径
    base = next((r for r in rows if r[0] == 1 and r[1] == 150), rows[0])
    ok = [r for r in rows if r[3] <= base[3] * 1.2]
    best = min(ok, key=lambda r: r[2]['step']) if ok else min(rows, key=lambda r: r[2]['step'])
    print(f'\n[选优] 基线 stride=1/mc=150 位置 {base[3]:.3f} m、step {base[2]["step"]:.1f} ms')
    print(f'       约束「位置 RMS ≤ 基线的 1.2 倍」下最快：stride={best[0]} max_cnt={best[1]} '
          f'→ step {best[2]["step"]:.1f} ms（{100*best[2]["step"]/base[2]["step"]:.0f}% 基线），'
          f'位置 {best[3]:.3f} m')
