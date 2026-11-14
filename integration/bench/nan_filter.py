import rclpy, numpy as np
from sensor_msgs.msg import PointCloud2
rclpy.init()
n = rclpy.create_node("nan_filter")
pub = n.create_publisher(PointCloud2, "/rslidar_clean", 10)
stat = {"in": 0, "bad": 0}
def cb(m):
    ps = m.point_step
    raw = np.frombuffer(bytes(m.data), dtype=np.uint8)
    npts = len(raw) // ps
    arr = raw[:npts*ps].reshape(npts, ps).copy()
    x = arr[:, 0:4].copy().view(np.float32).ravel()
    y = arr[:, 4:8].copy().view(np.float32).ravel()
    z = arr[:, 8:12].copy().view(np.float32).ravel()
    bad = ~(np.isfinite(x) & np.isfinite(y) & np.isfinite(z))
    if bad.any():
        arr[bad, 0:12] = 0
        stat["bad"] += int(bad.sum())
    stat["in"] += 1
    out = PointCloud2()
    out.header = m.header
    out.height = 1; out.width = npts
    out.fields = m.fields
    out.is_bigendian = m.is_bigendian
    out.point_step = ps; out.row_step = ps * npts
    out.is_dense = False
    out.data = arr.tobytes()
    pub.publish(out)
    if stat["in"] % 50 == 0:
        n.get_logger().info("frames=%d bad_points=%d" % (stat["in"], stat["bad"]))
n.create_subscription(PointCloud2, "/rslidar_points", cb, 10)
n.get_logger().info("nan_filter ready")
rclpy.spin(n)
