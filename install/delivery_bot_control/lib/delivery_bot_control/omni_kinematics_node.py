#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class OmniKinematicsNode(Node):
    def __init__(self):
        super().__init__('omni_kinematics')

        # ---- Parameters ----
        self.declare_parameter('wheel_radius', 0.024)            # m
        self.declare_parameter('robot_radius', 0.1155)           # m
        self.declare_parameter('wheel_angles_deg', [150.0, 30.0, 270.0])
        self.declare_parameter('cmd_rate', 50.0)                 # Hz
        self.declare_parameter('cmd_timeout', 0.5)               # s

        # Limits
        self.declare_parameter('max_wheel_accel', 20.0)          # rad/s^2 (per wheel)
        self.declare_parameter('max_wheel_speed', 100.0)         # rad/s clamp
        self.declare_parameter('max_wheel_jerk', 0.0)            # rad/s^3 (used only in per_wheel mode)

        # Ramping strategy
        #   'body_coordinated' : NEW - maintains body motion direction (recommended)
        #   'coordinated'  : ensures all wheels maintain ratio (still has temp yaw)
        #   'scalar'       : single progress variable s in [0..1], w_cmd = s * w_tgt (arrive together)
        #   'body_space'   : limit [vx,vy,wz] then map to wheels
        #   'wheel_vector' : scale entire wheel-delta vector uniformly
        #   'per_wheel'    : legacy independent clips (not recommended)
        self.declare_parameter('ramp_mode', 'body_coordinated')

        # Optional: wheel order & sign (if wiring flips a motor)
        self.declare_parameter('wheel_joint_names',
                               ['wheel1_joint', 'wheel2_joint', 'wheel3_joint'])
        self.declare_parameter('sign_correction', [1.0, 1.0, 1.0])

        # ---- Load params ----
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('robot_radius').value)
        angles_deg = list(self.get_parameter('wheel_angles_deg').value)
        phi = np.deg2rad(np.array(angles_deg, dtype=float))

        self.dt = 1.0 / float(self.get_parameter('cmd_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_acc = float(self.get_parameter('max_wheel_accel').value)
        self.max_w = float(self.get_parameter('max_wheel_speed').value)
        self.max_jerk = float(self.get_parameter('max_wheel_jerk').value)
        self.ramp_mode = str(self.get_parameter('ramp_mode').value)

        self.joint_names = list(self.get_parameter('wheel_joint_names').value)
        sign = np.array(list(self.get_parameter('sign_correction').value), dtype=float)
        self.S = np.diag(sign)

        # ---- Kinematics ----
        # v_rim = M @ [vx, vy, wz], omega = v_rim / r
        self.M = np.array([
            [-np.sin(phi[0]), np.cos(phi[0]), self.L],
            [-np.sin(phi[1]), np.cos(phi[1]), self.L],
            [-np.sin(phi[2]), np.cos(phi[2]), self.L]
        ], dtype=float)
        self.A = self.M / self.r                 # body -> wheels
        self.A_eff = self.S @ self.A             # include sign correction
        self.A_eff_pinv = np.linalg.pinv(self.A_eff)

        self.get_logger().info(f'Kinematic matrix M:\n{self.M}')
        self.get_logger().info(f'Ramp mode: {self.ramp_mode}')

        # ---- I/O ----
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/omni_wheel_drive_controller/commands', 10)

        # Optional debug publishers (body-space views)
        self.pub_cmd_body  = self.create_publisher(Twist, '/omni/debug/body_cmd_from_wheels', 10)
        self.pub_des_body  = self.create_publisher(Twist, '/omni/debug/body_desired', 10)
        self.pub_meas_body = self.create_publisher(Twist, '/omni/debug/body_measured_from_joint_states', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.last_meas_w = np.zeros(3, dtype=float)

        # ---- State ----
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9
        self.prev_w = np.zeros(3, dtype=float)
        # Scalar progress state for 'scalar' mode
        self.s = 0.0
        self.w_tgt_prev = np.zeros(3, dtype=float)
        
        # Coordinated motion state
        self.coord_velocity_scale = 0.0  # current scale factor [0,1]
        self.body_cmd_prev = np.zeros(3, dtype=float)  # [vx, vy, wz] for body coordinated mode

        # ---- Timer ----
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f'Omni ready: r={self.r:.4f} m, L={self.L:.4f} m, '
            f'angles(deg)={angles_deg}, rate={1.0/self.dt:.0f} Hz'
        )

    # ---------------- subs / helpers ----------------
    def _on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9
        # self.get_logger().info(f"RECEIVED CMD_VEL: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.angular.z:.3f}")

    def _on_joint_states(self, msg: JointState):
        try:
            vmap = dict(zip(msg.name, msg.velocity))
            got = [float(vmap[n]) for n in self.joint_names]
            self.last_meas_w = np.array(got, dtype=float)
        except Exception:
            pass

    def _pub_body_twist(self, pub, v3: np.ndarray):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(v3[0]), float(v3[1]), float(v3[2])
        pub.publish(t)

    def _target_wheels(self, vx: float, vy: float, wz: float) -> np.ndarray:
        v_body = np.array([vx, vy, wz], dtype=float)
        w_result = self.A_eff @ v_body
        # self.get_logger().info(f"KINEMATICS: body=[{vx:.3f}, {vy:.3f}, {wz:.3f}] -> wheels=[{w_result[0]:.3f}, {w_result[1]:.3f}, {w_result[2]:.3f}]")
        return w_result

    def _scale_vector_inf(self, prev: np.ndarray, tgt: np.ndarray, max_delta_inf: float) -> np.ndarray:
        delta = tgt - prev
        inf_n = float(np.max(np.abs(delta)))
        if inf_n == 0.0 or inf_n <= max_delta_inf:
            return tgt
        s = max_delta_inf / inf_n
        return prev + s * delta

    def _limit_body(self, v_prev: np.ndarray, v_tgt: np.ndarray, dv_max_lin: float, dw_max: float) -> np.ndarray:
        dv = v_tgt[:2] - v_prev[:2]
        dlin = float(np.linalg.norm(dv, ord=2))
        if dlin > dv_max_lin and dlin > 0.0:
            dv *= (dv_max_lin / dlin)
        dw = np.clip(v_tgt[2] - v_prev[2], -dw_max, dw_max)
        v_cmd = np.empty(3, dtype=float)
        v_cmd[:2] = v_prev[:2] + dv
        v_cmd[2]  = v_prev[2] + dw
        return v_cmd

    # -------------- body-coordinated motion profile --------------
    def _update_body_coordinated_motion(self, vx: float, vy: float, wz: float) -> np.ndarray:
        """
        Maintains body motion direction by limiting acceleration in body space,
        then mapping to wheels. This prevents temporary yaw during acceleration.
        """
        v_tgt = np.array([vx, vy, wz], dtype=float)
        max_tgt_mag = float(np.linalg.norm(v_tgt[:2]))  # linear velocity magnitude
        
        # self.get_logger().info(f"Body coordinated: v_tgt={v_tgt}, linear_mag={max_tgt_mag:.3f}")
        
        # If target is essentially zero, ramp down proportionally
        if max_tgt_mag < 1e-6 and abs(v_tgt[2]) < 1e-6:
            max_prev_mag = float(np.linalg.norm(self.body_cmd_prev[:2])) + abs(self.body_cmd_prev[2])
            if max_prev_mag < 1e-6:
                # self.get_logger().info("Both target and previous body commands are zero")
                self.body_cmd_prev = np.zeros(3, dtype=float)
                return np.zeros(3, dtype=float)
            
            # Decelerate in body space
            decel_rate = self.max_acc * self.dt / self.L * self.r  # Convert wheel accel to body accel
            decel_scale = min(1.0, decel_rate / max_prev_mag)
            v_cmd = self.body_cmd_prev * (1.0 - decel_scale)
            # self.get_logger().info(f"Body decel: prev={self.body_cmd_prev}, scale={decel_scale:.3f}, result={v_cmd}")
            self.body_cmd_prev = v_cmd
            return self.A_eff @ v_cmd

        # Calculate required body acceleration
        v_delta = v_tgt - self.body_cmd_prev
        
        # Separate linear and angular components for different acceleration limits
        linear_delta = v_delta[:2]
        angular_delta = v_delta[2]
        
        linear_delta_mag = float(np.linalg.norm(linear_delta))
        angular_delta_mag = float(abs(angular_delta))
        
        # self.get_logger().info(f"Body deltas: linear_mag={linear_delta_mag:.3f}, angular_mag={angular_delta_mag:.3f}")
        
        # Convert wheel acceleration limits to body space limits
        # For linear motion: max wheel speed change / effective radius
        max_linear_accel = (self.max_acc * self.dt * self.r) / self.L  # m/s per tick
        max_angular_accel = (self.max_acc * self.dt * self.r) / self.L  # rad/s per tick
        
        # Limit linear acceleration
        if linear_delta_mag > max_linear_accel and linear_delta_mag > 0:
            linear_scale = max_linear_accel / linear_delta_mag
            linear_delta_limited = linear_delta * linear_scale
            # self.get_logger().info(f"Linear limited: scale={linear_scale:.3f}")
        else:
            linear_delta_limited = linear_delta
            
        # Limit angular acceleration  
        angular_delta_limited = np.clip(angular_delta, -max_angular_accel, max_angular_accel)
        
        # Combine limited deltas
        v_cmd = self.body_cmd_prev.copy()
        v_cmd[:2] += linear_delta_limited
        v_cmd[2] += angular_delta_limited
        
        # self.get_logger().info(f"Body command: prev={self.body_cmd_prev}, cmd={v_cmd}")
        
        # Update state
        self.body_cmd_prev = v_cmd
        
        # Convert to wheel commands
        w_cmd = self.A_eff @ v_cmd
        # self.get_logger().info(f"Body->wheels: {v_cmd} -> {w_cmd}")
        
        return w_cmd
    def _update_coordinated_motion(self, w_tgt: np.ndarray) -> np.ndarray:
        """
        Ensures all wheels maintain their velocity ratios during acceleration.
        The limiting wheel (highest acceleration requirement) sets the pace.
        """
        # Debug logging
        max_tgt_mag = float(np.max(np.abs(w_tgt)))
        # self.get_logger().info(f"Coordinated motion: w_tgt={w_tgt}, max_tgt_mag={max_tgt_mag:.3f}")
        
        # If target is zero, ramp down to zero
        if max_tgt_mag < 1e-6:
            # Ramp down all wheels proportionally
            max_prev_mag = float(np.max(np.abs(self.prev_w)))
            if max_prev_mag < 1e-6:
                # self.get_logger().info("Both target and previous are zero, returning zeros")
                return np.zeros(3, dtype=float)
            
            # Determine maximum deceleration step
            d_w_max = self.max_acc * self.dt
            decel_scale = min(1.0, d_w_max / max_prev_mag)
            result = self.prev_w * (1.0 - decel_scale)
            # self.get_logger().info(f"Decelerating: prev_w={self.prev_w}, decel_scale={decel_scale:.3f}, result={result}")
            return result

        # Calculate required acceleration for each wheel to reach target
        w_delta = w_tgt - self.prev_w
        max_delta_mag = float(np.max(np.abs(w_delta)))
        
        # self.get_logger().info(f"w_delta={w_delta}, max_delta_mag={max_delta_mag:.3f}")
        
        # If we're already at target, return target
        if max_delta_mag < 1e-6:
            self.get_logger().info("Already at target")
            return w_tgt
        
        # Limit the step size based on maximum allowed acceleration
        d_w_max = self.max_acc * self.dt
        # self.get_logger().info(f"d_w_max={d_w_max:.3f}")
        
        # Scale all wheel deltas proportionally so the largest stays within limits
        if max_delta_mag > d_w_max:
            scale_factor = d_w_max / max_delta_mag
        else:
            scale_factor = 1.0
        
        # self.get_logger().info(f"scale_factor={scale_factor:.3f}")
        
        # Apply the coordinated step
        w_cmd = self.prev_w + scale_factor * w_delta
        
        # self.get_logger().info(f"Final w_cmd={w_cmd}")
        return w_cmd

    # -------------- scalar progress profile (improved) --------------
    def _update_scalar_progress_improved(self, w_tgt: np.ndarray) -> np.ndarray:
        """
        Improved scalar progress that handles direction changes better.
        """
        max_tgt_mag = float(np.max(np.abs(w_tgt)))
        
        # If target is essentially zero, ramp down
        if max_tgt_mag < 1e-6:
            self.coord_velocity_scale = max(0.0, self.coord_velocity_scale - self.max_acc * self.dt / 10.0)
            return self.coord_velocity_scale * self.w_tgt_prev if np.any(self.w_tgt_prev) else np.zeros(3)
        
        # Check if direction changed significantly
        if np.any(self.w_tgt_prev):
            # Normalize both vectors for comparison
            w_tgt_norm = w_tgt / max_tgt_mag
            prev_max_mag = float(np.max(np.abs(self.w_tgt_prev)))
            if prev_max_mag > 1e-6:
                w_prev_norm = self.w_tgt_prev / prev_max_mag
                dot_product = float(np.dot(w_tgt_norm, w_prev_norm))
                
                # If direction changed significantly, reset scale
                if dot_product < 0.9:  # ~25 degree threshold
                    self.coord_velocity_scale = 0.0
                    self.get_logger().debug(f"Direction change detected, resetting scale. Dot product: {dot_product:.3f}")
        
        # Store new target direction
        self.w_tgt_prev = w_tgt.copy()
        
        # Calculate maximum allowed acceleration step
        d_w_max = self.max_acc * self.dt
        max_scale_delta = d_w_max / max_tgt_mag
        
        # Ramp towards full scale (1.0)
        target_scale = 1.0
        scale_delta = np.clip(target_scale - self.coord_velocity_scale, -max_scale_delta, max_scale_delta)
        self.coord_velocity_scale += scale_delta
        
        # Ensure scale stays in valid range
        self.coord_velocity_scale = np.clip(self.coord_velocity_scale, 0.0, 1.0)
        
        return self.coord_velocity_scale * w_tgt

    def _update_scalar_progress(self, w_tgt: np.ndarray) -> float:
        """
        Original scalar progress - kept for compatibility
        """
        # If the target direction changed, re-project current command onto new direction
        if not np.allclose(w_tgt, self.w_tgt_prev, atol=1e-6):
            denom = float(np.dot(w_tgt, w_tgt))
            if denom > 1e-6:
                s_proj = float(np.dot(self.prev_w, w_tgt) / denom)
            else:
                s_proj = 0.0
            self.s = max(0.0, s_proj)  # Don't allow negative projection
            self.w_tgt_prev = w_tgt.copy()

        # Per-tick wheel delta cap
        d_w_max = self.max_acc * self.dt

        # ds_max so that the wheel with the largest magnitude hits d_w_max
        max_mag = float(np.max(np.abs(w_tgt)))
        ds_max = d_w_max / max_mag if max_mag > 1e-6 else 1.0

        # Move s toward 1.0 (or 0.0 if command is zero) with |ds| <= ds_max
        s_goal = 1.0 if max_mag > 1e-6 else 0.0
        ds = np.clip(s_goal - self.s, -ds_max, ds_max)
        self.s += ds
        self.s = np.clip(self.s, 0.0, 1.0)  # Ensure valid range
        return self.s

    # ----------------------------- Tick ------------------------------
    def _tick(self):
        # Timeout safety -> zero command
        now = self.get_clock().now().nanoseconds * 1e-9
        cmd_age = now - self.last_cmd_time
        if cmd_age > self.cmd_timeout:
            vx = vy = wz = 0.0
            # self.get_logger().warn(f"CMD TIMEOUT: age={cmd_age:.3f}s > {self.cmd_timeout}s, using zero command")
        else:
            vx = float(self.last_cmd.linear.x)
            vy = float(self.last_cmd.linear.y) 
            wz = float(self.last_cmd.angular.z)
            # self.get_logger().info(f"CMD ACTIVE: age={cmd_age:.3f}s, using vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")

        # Desired wheels from desired twist
        w_tgt = self._target_wheels(vx, vy, wz)

        # Compute command per selected ramping mode
        mode = self.ramp_mode
        if mode == 'body_coordinated':
            # NEW: Maintains body motion direction during acceleration
            w_cmd = self._update_body_coordinated_motion(vx, vy, wz)
            
        elif mode == 'coordinated':
            # Coordinated motion that maintains wheel velocity ratios
            w_cmd = self._update_coordinated_motion(w_tgt)
            
        elif mode == 'scalar_improved':
            # Improved scalar mode with better direction change handling
            w_cmd = self._update_scalar_progress_improved(w_tgt)

        elif mode == 'scalar':
            # Single scalar progress so all wheels arrive together
            s = self._update_scalar_progress(w_tgt)
            w_cmd = s * w_tgt

        elif mode == 'body_space':
            v_prev = self.A_eff_pinv @ self.prev_w
            v_tgt  = np.array([vx, vy, wz], dtype=float)
            d_w_max = self.max_acc * self.dt
            dv_max_lin = (self.r * d_w_max) / max(self.L, 1e-6)
            dw_max     = (self.r * d_w_max) / max(self.L, 1e-6)
            v_cmd = self._limit_body(v_prev, v_tgt, dv_max_lin, dw_max)
            w_cmd = self.A_eff @ v_cmd

        elif mode == 'wheel_vector':
            d_w_max = self.max_acc * self.dt
            w_cmd = self._scale_vector_inf(self.prev_w, w_tgt, d_w_max)

        else:  # 'per_wheel' (legacy)
            d_w_max = self.max_acc * self.dt
            if self.max_jerk > 0.0:
                alpha_des = (w_tgt - self.prev_w) / self.dt
                max_dalpha = self.max_jerk * self.dt
                alpha = np.clip(alpha_des, -self.max_acc, self.max_acc)
                alpha = np.clip(alpha, -max_dalpha, max_dalpha)
                w_cmd = self.prev_w + alpha * self.dt
            else:
                delta_w = np.clip(w_tgt - self.prev_w, -d_w_max, d_w_max)
                w_cmd = self.prev_w + delta_w

        # Clamp to max wheel speed for safety
        w_cmd = np.clip(w_cmd, -self.max_w, self.max_w)

        # -------- Debug: publish body twists --------
        v_cmd_body = self.A_eff_pinv @ w_cmd
        v_des_body = np.array([vx, vy, wz], dtype=float)
        self._pub_body_twist(self.pub_cmd_body, v_cmd_body)
        self._pub_body_twist(self.pub_des_body, v_des_body)
        if np.any(self.last_meas_w):
            v_meas_body = self.A_eff_pinv @ self.last_meas_w
            self._pub_body_twist(self.pub_meas_body, v_meas_body)

        # -------- Publish & update --------
        msg = Float64MultiArray()
        msg.data = w_cmd.astype(float).tolist()
        self.pub.publish(msg)
        # self.get_logger().info(f"PUBLISHED: [{msg.data[0]:.3f}, {msg.data[1]:.3f}, {msg.data[2]:.3f}] to {self.pub.topic_name}")
        self.prev_w = w_cmd.copy()

        # # Debug logging for troubleshooting
        # max_w_tgt = float(np.max(np.abs(w_tgt)))
        # max_w_cmd = float(np.max(np.abs(w_cmd)))
        # self.get_logger().info(
        #     f"TICK: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f} | "
        #     f"Mode: {mode} | w_tgt: [{w_tgt[0]:.2f}, {w_tgt[1]:.2f}, {w_tgt[2]:.2f}] | "
        #     f"w_cmd: [{w_cmd[0]:.2f}, {w_cmd[1]:.2f}, {w_cmd[2]:.2f}] | "
        #     f"cmd_age: {now - self.last_cmd_time:.3f}s"
        # )

def main():
    rclpy.init()
    node = OmniKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()