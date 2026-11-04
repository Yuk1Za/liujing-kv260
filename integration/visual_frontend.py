"""同步 RGB-D 到视觉位姿的接入接口；由相机采集进程调用 process。"""
from dataclasses import dataclass
from pathlib import Path
import os
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision/lvi_board"))
from vo_core import PnPVO

@dataclass
class RGBDFrame:
    stamp_ns: int
    gray: np.ndarray       # H x W uint8，已去畸变
    depth_m: np.ndarray    # H x W float32，已注册到 gray，单位米

@dataclass
class VisualPose:
    stamp_ns: int
    T_body0_body: np.ndarray
    inliers: int
    reprojection_rms_px: float

class VisualFrontend:
    """matcher 提供 match(gray_prev, gray_cur, det='st')；输出 uv_prev / uv_cur。"""
    def __init__(self, K, T_body_camera, matcher):
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.Tbc = np.asarray(T_body_camera, dtype=float).reshape(4, 4)
        self.Tcb = np.linalg.inv(self.Tbc)
        self.matcher = matcher
        self.vo = PnPVO(self.K)
        self.previous = None

    def process(self, frame):
        if frame.gray.ndim != 2 or frame.depth_m.shape != frame.gray.shape:
            raise ValueError("aligned gray/depth dimensions required")
        if self.previous is None:
            self.previous = frame
            return VisualPose(frame.stamp_ns, np.eye(4), 0, 0.0)
        if frame.stamp_ns <= self.previous.stamp_ns:
            return None
        match = self.matcher.match(self.previous.gray, frame.gray, det="st")
        up = np.asarray(match["uv_prev"], dtype=float).reshape(-1, 2)
        uc = np.asarray(match["uv_cur"], dtype=float).reshape(-1, 2)
        h, w = frame.gray.shape
        good = np.isfinite(up).all(axis=1) & np.isfinite(uc).all(axis=1)
        good &= (up[:, 0] >= 0) & (up[:, 0] < w-1) & (up[:, 1] >= 0) & (up[:, 1] < h-1)
        good &= (uc[:, 0] >= 0) & (uc[:, 0] < w) & (uc[:, 1] >= 0) & (uc[:, 1] < h)
        up, uc = up[good], uc[good]
        pixels = np.rint(up).astype(int)
        z = self.previous.depth_m[pixels[:, 1], pixels[:, 0]]
        good = np.isfinite(z) & (z > 0.1) & (z < 10)
        up, uc, z = up[good], uc[good], z[good]
        rays = np.column_stack((up, np.ones(len(up)))) @ np.linalg.inv(self.K).T
        xyz = rays * (z / rays[:, 2])[:, None]
        result = self.vo.step(up, uc, np.empty((0, 2)), np.empty(0), X3_pre=xyz)
        if result is None:
            return None
        self.previous = frame
        return VisualPose(frame.stamp_ns, self.Tbc @ self.vo.pose() @ self.Tcb,
                          result["n_inliers"], result["rms_px"])

def attach_final3(bit, K, T_body_camera, width=640, height=480):
    # 位流由顶层所有者加载一次，视觉分支复用现有 PL。
    os.environ["PL_DOWNLOAD"] = "0"
    os.environ["PL_CLEAR_CACHE"] = "0"
    from pl_match import PLMatcher
    matcher = PLMatcher(bit=str(bit), w=width, h=height, nms="on")
    return VisualFrontend(K, T_body_camera, matcher)

class VisualPosePublisher:
    """把 VisualPose 写入 final3 定位后端已订阅的 /vo_pnp/abs。"""
    def __init__(self, node, frame_id="camera_init"):
        from geometry_msgs.msg import PoseStamped
        self.msg_type = PoseStamped
        self.pub = node.create_publisher(PoseStamped, "/vo_pnp/abs", 50)
        self.frame_id = frame_id

    def publish(self, pose):
        from scipy.spatial.transform import Rotation
        msg = self.msg_type()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(pose.stamp_ns, 1000000000)
        msg.header.frame_id = self.frame_id
        t = pose.T_body0_body[:3, 3]
        q = Rotation.from_matrix(pose.T_body0_body[:3, :3]).as_quat()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, t)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, q)
        self.pub.publish(msg)
