"""Offline projective point-to-plane diagnostic; never certifies calibration.

Train and held-out directories must differ. Produces evidence and an explicitly
unaccepted candidate, without modifying the running configuration.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def load(folder, intr, limit=None):
    rows = []
    paths = sorted(Path(folder).glob('*.npz'))
    if limit and len(paths)>limit:
        paths = [paths[i] for i in np.linspace(0,len(paths)-1,limit,dtype=int)]
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            d, lidar = data['depth'].astype(float), data['lidar_xyz'].astype(float)
        y, x = np.indices(d.shape)
        q = np.stack(((x-intr['cx'])*d/intr['fx'], (y-intr['cy'])*d/intr['fy'], d), -1)
        n = np.zeros_like(q)
        n[2:-2, 2:-2] = np.cross(q[2:-2, 4:]-q[2:-2, :-4], q[4:, 2:-2]-q[:-4, 2:-2])
        norm = np.linalg.norm(n, axis=2)
        valid = (norm > 1e-8) & (d > .3) & (d < 5)
        for dx, dy in ((2, 0), (-2, 0), (0, 2), (0, -2)):
            other = np.roll(d, (dy, dx), axis=(0, 1))
            valid &= (other > .3) & (np.abs(d-other) < .03)
        n /= np.maximum(norm[..., None], 1e-12)
        rows.append((q, n, valid, lidar, path.name))
    if not rows:
        raise ValueError('no observations')
    return rows


def correspondences(rows, T, intr, gate=.25):
    points, normals, errors = [], [], []
    for q, n, mask, lidar, _ in rows:
        p = lidar @ T[:3, :3].T + T[:3, 3]
        p = p[np.isfinite(p).all(1) & (p[:, 2] > .3) & (p[:, 2] < 5)]
        uv = np.rint(p[:, :2]/p[:, 2:3] * [intr['fx'], intr['fy']] + [intr['cx'], intr['cy']]).astype(int)
        keep = (uv[:, 0] >= 2) & (uv[:, 0] < q.shape[1]-2) & (uv[:, 1] >= 2) & (uv[:, 1] < q.shape[0]-2)
        uv, p = uv[keep], p[keep]
        u, v = uv.T
        keep = mask[v, u] & (np.abs(p[:, 2]-q[v, u, 2]) < gate)
        p, nn, target = p[keep], n[v[keep], u[keep]], q[v[keep], u[keep]]
        points.append(p); normals.append(nn); errors.append(np.sum(nn*(p-target), axis=1))
    return np.concatenate(points), np.concatenate(normals), np.concatenate(errors)


def summary(rows, T, intr):
    _, _, e = correspondences(rows, T, intr)
    return dict(interior_gated_correspondences=len(e), median_abs_plane_m=float(np.median(abs(e))) if len(e) else None,
                p90_abs_plane_m=float(np.percentile(abs(e), 90)) if len(e) else None,
                warning='Gated interior subset; occluded/unmatched points excluded, not acceptance')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('train'); ap.add_argument('heldout'); ap.add_argument('--calibration', required=True); ap.add_argument('--out', required=True)
    a = ap.parse_args()
    if Path(a.train).resolve() == Path(a.heldout).resolve(): raise ValueError('separate scenes required')
    out = Path(a.out); out.mkdir(exist_ok=False)
    cal = json.loads(Path(a.calibration).read_text()); intr = cal['depth']
    train, heldout = load(a.train, intr), load(a.heldout, intr)
    initial = np.linalg.inv(np.asarray(cal['T_pose_depth'])); T = initial.copy(); history = []
    for iteration in range(40):
        p, n, e = correspondences(train, T, intr)
        if len(e) < 100: break
        J = np.concatenate((np.cross(p, n), n), axis=1); weight = np.minimum(1, .02/np.maximum(abs(e), 1e-12))
        H = (J.T*weight)@J; condition = float(np.linalg.cond(H))
        history.append(dict(iteration=iteration, median_m=float(np.median(abs(e))), condition=condition, samples=len(e)))
        if not np.isfinite(condition) or condition > 1e6: break
        delta = np.linalg.solve(H, -(J.T*weight)@e)
        delta *= min(1., .02/max(np.linalg.norm(delta[:3]), 1e-12), .02/max(np.linalg.norm(delta[3:]), 1e-12))
        angle = np.linalg.norm(delta[:3]); x, y, z = delta[:3]/max(angle, 1e-12)
        K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        step = np.eye(4); step[:3, :3] += np.sin(angle)*K+(1-np.cos(angle))*(K@K); step[:3, 3] = delta[3:]
        T = step @ T
        if np.linalg.norm(delta) < 1e-6: break
    change = T @ np.linalg.inv(initial)
    report = dict(accepted=False, reason='Candidate fitting only; independent geometry, time and pose-axis verification still required',
                  train_before=summary(train, initial, intr), train_after=summary(train, T, intr),
                  heldout_before=summary(heldout, initial, intr), heldout_after=summary(heldout, T, intr),
                  translation_change_m=float(np.linalg.norm(change[:3, 3])),
                  rotation_change_deg=float(np.degrees(np.arccos(np.clip((np.trace(change[:3, :3])-1)/2, -1, 1)))), history=history)
    cal['T_pose_depth'] = np.linalg.inv(T).tolist(); cal['verified'] = {k: False for k in ('intrinsics', 'depth_color', 'pose_camera', 'time_alignment', 'pose_axes', 'corrected_pose')}
    cal['description'] = 'UNACCEPTED offline projective candidate; never installed'
    (out/'candidate.json').write_text(json.dumps(cal, indent=2)); (out/'report.json').write_text(json.dumps(report, indent=2)); print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
