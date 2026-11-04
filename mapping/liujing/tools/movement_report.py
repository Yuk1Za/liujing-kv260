"""Report measured pose motion without promoting calibration/acceptance flags."""
import argparse
import json
import math
import statistics
from pathlib import Path


def rotate_inverse(q, vector):
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-8:
        raise ValueError('invalid quaternion')
    x, y, z, w = [v/norm for v in q]
    R = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    return [sum(R[j][i]*vector[j] for j in range(3)) for i in range(3)]


def report(rows, calibration):
    result = {'accepted': False, 'scope': 'Internal trajectory estimates; physical travel/rotation and camera timing require independent checks', 'streams': {}}
    R = calibration['T_pose_depth']
    for kind in ('raw', 'mapped'):
        poses = [r for r in rows if r['kind'] == kind]
        if len(poses) < 2:
            result['streams'][kind] = {'samples': len(poses)}
            continue
        start, end = poses[0]['elapsed_s'], poses[-1]['elapsed_s']
        first = [r for r in poses if r['elapsed_s'] <= start+2]
        last = [r for r in poses if r['elapsed_s'] >= end-2]
        origin = [statistics.median(r['position'][i] for r in first) for i in range(3)]
        finish = [statistics.median(r['position'][i] for r in last) for i in range(3)]
        distances, camera_displacements, angles = [], [], []
        initial_q = poses[0]['quaternion_xyzw']
        for row in poses:
            delta = [row['position'][i]-origin[i] for i in range(3)]
            distances.append(math.sqrt(sum(v*v for v in delta)))
            local = rotate_inverse(initial_q, delta)
            camera_displacements.append([sum(R[j][i]*local[j] for j in range(3)) for i in range(3)])
            q = row['quaternion_xyzw']
            dot = abs(sum(a*b for a,b in zip(q, initial_q))) / math.sqrt(sum(v*v for v in q)*sum(v*v for v in initial_q))
            angles.append(math.degrees(2*math.acos(min(1, dot))))
        ages = sorted(r['source_age_s'] for r in poses)
        result['streams'][kind] = dict(samples=len(poses), duration_s=end-start,
            maximum_distance_from_start_m=max(distances),
            final_offset_from_start_m=math.sqrt(sum((finish[i]-origin[i])**2 for i in range(3))),
            maximum_rotation_from_start_deg=max(angles),
            initial_camera_axis_ranges_m={axis: [min(p[i] for p in camera_displacements), max(p[i] for p in camera_displacements)] for i,axis in enumerate(('right','down','forward'))},
            source_age_p95_s=ages[min(len(ages)-1, math.ceil(len(ages)*.95)-1)],
            maximum_arrival_gap_s=max(b['elapsed_s']-a['elapsed_s'] for a,b in zip(poses,poses[1:])))
    return result


def main():
    p = argparse.ArgumentParser(); p.add_argument('run'); p.add_argument('--calibration', required=True)
    a = p.parse_args(); root = Path(a.run)
    rows = [json.loads(s) for s in (root/'poses.jsonl').read_text().splitlines() if s.strip()]
    result = report(rows, json.loads(Path(a.calibration).read_text()))
    (root/'movement_report.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
