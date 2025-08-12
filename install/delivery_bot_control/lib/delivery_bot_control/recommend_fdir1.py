#!/usr/bin/env python3
import rclpy, math, numpy as np
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

WHEELS = ["wheel1", "wheel2", "wheel3"]
BASE = "base_link"

def q_to_R(q):
    x,y,z,w = q.x, q.y, q.z, q.w
    # 3x3 rotation matrix (child->parent)
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ])

class FdirSuggest(Node):
    def __init__(self):
        super().__init__('fdir_suggest')
        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self)
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        try:
            for w in WHEELS:
                t: TransformStamped = self.buf.lookup_transform(BASE, w, rclpy.time.Time())
                # Wheel position in base frame (for radial dir)
                px, py = t.transform.translation.x, t.transform.translation.y
                r = np.array([px, py])
                if np.linalg.norm(r) < 1e-6:
                    self.get_logger().warn(f"{w}: zero radius? skipping")
                    continue
                radial = r/np.linalg.norm(r)
                # Tangent in base frame (CCW): [-ry, rx, 0]
                t_base = np.array([-radial[1], radial[0], 0.0])
                # Rotate to wheel frame: v_w = R^T * v_b (R maps wheel->base)
                R = q_to_R(t.transform.rotation)
                t_wheel = R.T @ t_base
                # Compare to wheel axes X=[1,0,0], Y=[0,1,0]
                dots = {
                    "+X": float(t_wheel[0]),
                    "+Y": float(t_wheel[1]),
                }
                choice = "X" if abs(dots["+X"]) >= abs(dots["+Y"]) else "Y"
                sign = "+" if dots[f"+{choice}"] >= 0 else "-"
                self.get_logger().info(f"{w}: recommend fdir1 = {sign}{choice}  "
                                       f"(t_wheel ~ {t_wheel.round(3)})")
            rclpy.shutdown()
        except Exception as e:
            # Wait for TF to become available
            pass

def main():
    rclpy.init()
    n = FdirSuggest()
    rclpy.spin(n)

if __name__ == "__main__":
    main()
