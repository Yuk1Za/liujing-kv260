"""Online scheduling/provenance checks, independent of ROS and hardware."""
import math
import numpy as np
from alignment import checked_pose, validate_calibration


def validate_session(calibration, mode):
    if mode not in ('calibrated', 'static_diagnostic'):
        raise ValueError('unknown session mode')
    validate_calibration(calibration, live=mode == 'calibrated')
    if mode == 'static_diagnostic' and not calibration.get('verified', {}).get('intrinsics'):
        raise ValueError('static diagnostic still requires measured camera intrinsics')


class StaticGuard:
    """Never substitutes identity pose: abort on motion relative to first pose."""
    def __init__(self, translation=.02, rotation=math.radians(2)):
        self.anchor = None
        self.translation = translation
        self.rotation = rotation

    def check(self, pose):
        pose = checked_pose(pose)
        if self.anchor is None:
            self.anchor = pose.copy()
        distance = float(np.linalg.norm(pose[:3, 3] - self.anchor[:3, 3]))
        angle = float(np.arccos(np.clip((np.trace(self.anchor[:3, :3].T @ pose[:3, :3]) - 1) / 2, -1, 1)))
        if distance > self.translation or angle > self.rotation:
            raise ValueError('static diagnostic motion limit exceeded; calibration required')
        return distance, angle


def pose_support(items, stamp, max_gap):
    """Return actual source times supporting an interpolated pose."""
    rows = list(items)
    for ts, _ in rows:
        if abs(ts - stamp) < 1e-7:
            return [ts, ts]
    for (ta, _), (tb, _) in zip(rows, rows[1:]):
        if ta < stamp < tb and tb - ta <= max_gap:
            return [ta, tb]
    return None


def finite_rate(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('rate must be positive and finite')
    return value
