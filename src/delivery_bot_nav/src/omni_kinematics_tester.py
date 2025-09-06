#!/usr/bin/env python3
import rclpy
import math
import time
import csv
import os
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from collections import defaultdict
import matplotlib.pyplot as plt

# ================== USER-CONFIGURABLE PARAMETERS ================== #
# --- Topics (change here) ---
CMD_VEL_TOPIC = '/omni_drive_controller/cmd_vel'                         # publishes test TwistStamped
JOINT_STATE_TOPIC = '/joint_states'                # wheel encoder velocities
ODOM_TWIST_TOPIC = '/omni_drive_controller/odom'   # nav_msgs/Odometry (for odom yaw rate)
IMU_TOPIC = '/imu'                            # sensor_msgs/Imu

# --- Wheel/joint model (ORDER MATTERS) ---
WHEEL_JOINT_NAMES = ['wheel1_joint', 'wheel2_joint', 'wheel3_joint']

# --- Robot geometry ---
ROBOT_RADIUS = 0.115    # [m] center -> wheel contact
WHEEL_RADIUS = 0.024     # [m]
WHEEL_ANGLES_DEG = [30, 150, 270]  # [deg] CCW from +X

# --- Output files/plots ---
CSV_DIR = '/home/abanoub/delivery_bot_ws/src/delivery_bot_nav/src'
os.makedirs(CSV_DIR, exist_ok=True)

# --- Motion test suite (vx, vy, wz) in base_link ---
MOTION_TESTS = {
    'X+': (0.2, 0.0, 0.0),
    'X-': (-0.2, 0.0, 0.0),
    'Y+': (0.0, 0.2, 0.0),
    'Y-': (0.0, -0.2, 0.0),
    'Diagonal1': (0.2, 0.2, 0.0),
    'Diagonal2': (-0.2, -0.2, 0.0),
    'Diagonal3': (0.2, -0.2, 0.0),
    'Diagonal4': (-0.2, 0.2, 0.0),
    'YawCW': (0.0, 0.0, -0.5),
    'YawCCW': (0.0, 0.0, 0.5),
}

# --- Timing ---
TEST_DURATION = 6.0       # [s]
WARMUP_DURATION = 1.0     # [s] wait before sampling
SAMPLE_INTERVAL = 0.2     # [s] 5 Hz sampling (≈25 samples)
STOP_DURATION = 1.0       # [s] stop between tests

# --- Yaw sign check options ---
ENABLE_YAW_SIGN_CHECK = True     # enable/disable the print + file
YAW_SIGN_EPS = 0.05              # [rad/s] deadband for sign decision
WRITE_YAW_SIGN_TEXT = True       # also write yaw_sign_check.txt per yaw test
# ================================================================ #

def omni_forward_kinematics(vx, vy, wz):
    """Expected wheel speeds (rad/s) from body twist for 3-wheel omni."""
    out = []
    for ang_deg in WHEEL_ANGLES_DEG:
        th = math.radians(ang_deg)
        out.append(( math.sin(th) * vx - math.cos(th) * vy - ROBOT_RADIUS * wz ) / WHEEL_RADIUS)
    return out

def reconstruct_wz_from_joint_states(wheel_speeds):
    """
    During pure yaw (vx=vy=0): wi = -(R * wz) / r  => wz = -wi * r / R.
    Average across wheels for robustness.
    """
    if not wheel_speeds:
        return 0.0
    ests = [(-w * WHEEL_RADIUS / ROBOT_RADIUS) for w in wheel_speeds]
    return sum(ests) / len(ests)

def sign_to_label(value, eps):
    if value > eps:
        return 'CCW'
    if value < -eps:
        return 'CW'
    return 'ZERO'

class Debugger(Node):
    def __init__(self):
        super().__init__('omni_debugger_node')

        # QoS profiles for better reliability
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, reliable_qos)
        self.js_sub = self.create_subscription(JointState, JOINT_STATE_TOPIC, self.joint_cb, sensor_qos)
        self.imu_sub = self.create_subscription(Imu, IMU_TOPIC, self.imu_cb, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TWIST_TOPIC, self.odom_cb, sensor_qos)

        # Thread-safe data storage
        self.data_lock = threading.Lock()
        
        # Caches
        self.current_joint_vels = {}   # name -> vel
        self.js_received = False
        self.js_timestamp = 0.0

        self.last_imu_wz = 0.0
        self.imu_received = False
        self.imu_timestamp = 0.0

        self.last_odom_wz = 0.0
        self.odom_received = False
        self.odom_timestamp = 0.0

        self.final_results = {}  # per-test average errors

        # Timer for spinning
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        
        self.get_logger().info("Omni Debugger initialized")
        self.get_logger().info(f"Publishing to: {CMD_VEL_TOPIC}")
        self.get_logger().info(f"Listening to joint states: {JOINT_STATE_TOPIC}")
        self.get_logger().info(f"Listening to IMU: {IMU_TOPIC}")
        self.get_logger().info(f"Listening to odometry: {ODOM_TWIST_TOPIC}")

    def timer_callback(self):
        """Keep the node spinning"""
        pass

    # ---- Callbacks ----
    def joint_cb(self, msg: JointState):
        with self.data_lock:
            for name, vel in zip(msg.name, msg.velocity):
                if name in WHEEL_JOINT_NAMES:
                    self.current_joint_vels[name] = vel
            self.js_received = True
            self.js_timestamp = time.time()

    def imu_cb(self, msg: Imu):
        with self.data_lock:
            self.last_imu_wz = float(msg.angular_velocity.z)
            self.imu_received = True
            self.imu_timestamp = time.time()

    def odom_cb(self, msg: Odometry):
        with self.data_lock:
            self.last_odom_wz = float(msg.twist.twist.angular.z)
            self.odom_received = True
            self.odom_timestamp = time.time()

    def publish_stop_command(self):
        """Publish zero velocity command"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def wait_for_topics(self, timeout=10.0):
        """Wait for all topics to be available"""
        self.get_logger().info("Waiting for topics to be available...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self.data_lock:
                if self.js_received and self.imu_received and self.odom_received:
                    self.get_logger().info("All topics are now available!")
                    return True
            
            # Spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if (time.time() - start_time) % 2.0 < 0.1:  # Log every 2 seconds
                with self.data_lock:
                    self.get_logger().info(f"Status: JS={self.js_received}, IMU={self.imu_received}, ODOM={self.odom_received}")
        
        self.get_logger().warn("Timeout waiting for all topics. Proceeding with available data.")
        return False

    # ---- Core test runner ----
    def run_test(self, test_name, vx, vy, wz):
        self.get_logger().info(f"Running test: {test_name} (vx={vx}, vy={vy}, wz={wz})")
        
        twist = TwistStamped()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.angular.z = wz

        expected_wheels = omni_forward_kinematics(vx, vy, wz)
        self.get_logger().info(f"Expected wheel speeds: {[f'{w:.3f}' for w in expected_wheels]} rad/s")

        # Buffers
        samples = []
        timestamps = []
        imu_wz_series, odom_wz_series, js_wz_series = [], [], []

        # Warmup phase
        self.get_logger().info(f"Warmup phase ({WARMUP_DURATION}s)...")
        start_time = time.time()
        while time.time() - start_time < WARMUP_DURATION:
            twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        # Data collection phase
        self.get_logger().info(f"Data collection phase ({TEST_DURATION - WARMUP_DURATION}s)...")
        sample_count = 0
        while time.time() - start_time < TEST_DURATION:
            # Publish command
            twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(twist)

            # Spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.01)

            # Collect sample if joint states are available
            with self.data_lock:
                if self.js_received and len(self.current_joint_vels) >= len(WHEEL_JOINT_NAMES):
                    cmd_wheel = expected_wheels
                    actual_wheel = [self.current_joint_vels.get(n, 0.0) for n in WHEEL_JOINT_NAMES]
                    error = [abs(a - e) for a, e in zip(actual_wheel, cmd_wheel)]
                    samples.append((cmd_wheel, actual_wheel, error))
                    timestamps.append(time.time() - start_time)
                    sample_count += 1

                    # Collect yaw sign streams only for yaw tests
                    if ENABLE_YAW_SIGN_CHECK and test_name.startswith('Yaw'):
                        if self.imu_received:
                            imu_wz_series.append(self.last_imu_wz)
                        if self.odom_received:
                            odom_wz_series.append(self.last_odom_wz)
                        js_wz_series.append(reconstruct_wz_from_joint_states(actual_wheel))

            time.sleep(SAMPLE_INTERVAL)

        self.get_logger().info(f"Collected {sample_count} samples")

        # Stop the robot
        self.get_logger().info("Stopping robot...")
        for _ in range(10):  # Send stop commands
            self.publish_stop_command()
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        # Save results
        if samples:
            self.save_csv(test_name, samples, timestamps)
            self.plot(test_name, samples, timestamps)
        else:
            self.get_logger().warn(f"No samples collected for test {test_name}")

        # Yaw sign analysis
        if ENABLE_YAW_SIGN_CHECK and test_name.startswith('Yaw') and samples:
            imu_mean = (sum(imu_wz_series) / len(imu_wz_series)) if imu_wz_series else 0.0
            odom_mean = (sum(odom_wz_series) / len(odom_wz_series)) if odom_wz_series else 0.0
            js_mean = (sum(js_wz_series) / len(js_wz_series)) if js_wz_series else 0.0

            imu_label = sign_to_label(imu_mean, YAW_SIGN_EPS)
            odom_label = sign_to_label(odom_mean, YAW_SIGN_EPS)
            js_label = sign_to_label(js_mean, YAW_SIGN_EPS)

            self.get_logger().info(f"[{test_name}] imu: {imu_label} | odom: {odom_label} | joint_states: {js_label}")

            if WRITE_YAW_SIGN_TEXT:
                test_dir = os.path.join(CSV_DIR, test_name)
                os.makedirs(test_dir, exist_ok=True)
                with open(os.path.join(test_dir, 'yaw_sign_check.txt'), 'w') as f:
                    f.write(f"imu_mean_wz={imu_mean:.4f} -> {imu_label}\n")
                    f.write(f"odom_mean_wz={odom_mean:.4f} -> {odom_label}\n")
                    f.write(f"joint_states_mean_wz={js_mean:.4f} -> {js_label}\n")
                    f.write(f"eps={YAW_SIGN_EPS:.3f}\n")

        # Wait between tests
        if STOP_DURATION > 0:
            self.get_logger().info(f"Waiting {STOP_DURATION}s between tests...")
            time.sleep(STOP_DURATION)

    # ---- CSV / plotting / summaries ----
    def save_csv(self, test_name, samples, timestamps):
        test_dir = os.path.join(CSV_DIR, test_name)
        os.makedirs(test_dir, exist_ok=True)

        # Time series data
        with open(os.path.join(test_dir, 'samples.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            header = ['Time']
            for j in WHEEL_JOINT_NAMES:
                header += [f'{j}_expected', f'{j}_actual', f'{j}_error']
            w.writerow(header)
            for t, (exp, act, err) in zip(timestamps, samples):
                row = [round(t, 2)]
                for e, a, er in zip(exp, act, err):
                    row += [round(e, 6), round(a, 6), round(er, 6)]
                w.writerow(row)

        # Per-test summary
        with open(os.path.join(test_dir, 'summary.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['Wheel', 'Average Error', 'Max Error', 'RMS Error'])
            for i, name in enumerate(WHEEL_JOINT_NAMES):
                errors = [s[2][i] for s in samples]
                avg_err = sum(errors) / len(errors) if errors else 0.0
                max_err = max(errors) if errors else 0.0
                rms_err = math.sqrt(sum(e*e for e in errors) / len(errors)) if errors else 0.0
                w.writerow([name, round(avg_err, 6), round(max_err, 6), round(rms_err, 6)])

        # Store for global summary
        if samples:
            self.final_results[test_name] = {
                name: (sum(s[2][i] for s in samples) / len(samples))
                for i, name in enumerate(WHEEL_JOINT_NAMES)
            }

    def plot(self, test_name, samples, timestamps):
        if not samples:
            return
            
        plt.figure(figsize=(12, 10))
        
        # Plot wheel velocities
        plt.subplot(2, 1, 1)
        for i, name in enumerate(WHEEL_JOINT_NAMES):
            expected = [s[0][i] for s in samples]
            actual = [s[1][i] for s in samples]
            plt.plot(timestamps, expected, label=f'{name}_expected', linewidth=2)
            plt.plot(timestamps, actual, label=f'{name}_actual', linestyle='--', alpha=0.8)
        plt.title(f"Wheel Velocities - {test_name}")
        plt.xlabel("Time (s)")
        plt.ylabel("Wheel Velocity (rad/s)")
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Plot errors
        plt.subplot(2, 1, 2)
        for i, name in enumerate(WHEEL_JOINT_NAMES):
            errors = [s[2][i] for s in samples]
            plt.plot(timestamps, errors, label=f'{name}_error', marker='o', markersize=3)
        plt.title(f"Velocity Errors - {test_name}")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (rad/s)")
        plt.legend()
        plt.grid(True, alpha=0.3)

        plt.tight_layout()
        outdir = os.path.join(CSV_DIR, test_name)
        os.makedirs(outdir, exist_ok=True)
        plt.savefig(os.path.join(outdir, 'plot.png'), dpi=150, bbox_inches='tight')
        plt.close()

    def summarize_all(self):
        if not self.final_results:
            self.get_logger().warn("No test results to summarize")
            return

        groups = {
            'X': ['X+', 'X-'],
            'Y': ['Y+', 'Y-'],
            'Diagonals': ['Diagonal1', 'Diagonal2', 'Diagonal3', 'Diagonal4'],
            'Yaw': ['YawCW', 'YawCCW']
        }
        
        with open(os.path.join(CSV_DIR, 'group_summary.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['Group', 'Wheel', 'Average Error', 'Rank'])
            
            for group, tests in groups.items():
                wheel_errors = {j: [] for j in WHEEL_JOINT_NAMES}
                valid = [t for t in tests if t in self.final_results]
                if not valid:
                    continue
                    
                for tname in valid:
                    for j in WHEEL_JOINT_NAMES:
                        if j in self.final_results[tname]:
                            wheel_errors[j].append(self.final_results[tname][j])
                
                avgs = {j: (sum(v)/len(v) if v else float('nan')) for j, v in wheel_errors.items()}
                ranked = sorted(avgs.items(), key=lambda kv: kv[1] if not math.isnan(kv[1]) else float('inf'), reverse=True)
                
                for rank, (j, err) in enumerate(ranked, 1):
                    w.writerow([group, j, round(err, 6) if not math.isnan(err) else 'N/A', rank])

    def run(self):
        # Wait for topics to be available
        self.wait_for_topics()
        
        # Run all tests
        try:
            for name, (vx, vy, wz) in MOTION_TESTS.items():
                self.run_test(name, vx, vy, wz)
            
            # Generate summary
            self.summarize_all()
            self.get_logger().info("All tests complete.")
            
        except KeyboardInterrupt:
            self.get_logger().info("Tests interrupted by user")
        except Exception as e:
            self.get_logger().error(f"Test execution failed: {str(e)}")
        finally:
            # Ensure robot is stopped
            for _ in range(20):
                self.publish_stop_command()
                time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Debugger()
        
        # Run tests in a separate thread to allow callback processing
        import threading
        test_thread = threading.Thread(target=node.run)
        test_thread.start()
        
        # Keep spinning for callbacks
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        
        test_thread.join(timeout=1.0)
        
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()