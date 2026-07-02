#!/usr/bin/env python3
"""ESP32 <-> ROS 2 bridge for Plague-Bot VR (Adán's "Maestro" protocol).

ONE node owns the serial port because a single ESP32 multiplexes everything on
one line: encoder+IMU telemetry out, and D, (drive PWM) + A, (arm angles) in.

Telemetry IN  @50Hz:  E1,E2,E3,E4,AccX,AccY,AccZ,GyrX,GyrY,GyrZ\\n
  E1..E4 = raw encoder pulse counts (ints); Acc m/s^2; Gyro rad/s.
Drive   OUT:          D,M1_R,M1_L,M2_R,M2_L,M3_R,M3_L,M4_R,M4_L\\n   (8x PWM 0-255)
Arm     OUT:          A,J1,J2,J3,J4,J5,J6\\n                         (angles 0-180)

Publishes : nav_msgs/Odometry on ~odom_topic  (NO TF — the EKF owns odom->base)
            sensor_msgs/Imu   on ~imu_topic    (orientation marked unknown)
Subscribes: geometry_msgs/Twist on ~cmd_vel_topic  -> D, PWM
            std_msgs/Int16MultiArray on ~arm_topic  -> A, angles

Corner/sign mapping is UNKNOWN until calibrated with serial_sniffer + hand
spins, so EVERYTHING is a parameter. Defaults are a best guess (M1 front-left).
Verify against the physical wiring (ADR-0002 warns the sim swaps L/R) BEFORE
trusting autonomous motion.
"""
import math
import threading

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray

try:
    import serial  # pyserial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial no instalado: sudo apt install python3-serial"
    ) from exc


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Esp32Bridge(Node):
    def __init__(self) -> None:
        super().__init__("esp32_bridge")

        # --- Serial ----------------------------------------------------------
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 115200)

        # --- Frames / topics -------------------------------------------------
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("arm_topic", "/arm/joint_angles")

        # --- Kinematics (from URDF) ------------------------------------------
        self.declare_parameter("wheel_radius", 0.0762)
        self.declare_parameter("wheel_separation", 0.34)
        # Skid-steer scrub comp (sim used 1.65). Only affects reported odom yaw,
        # which the EKF ignores (it fuses IMU yaw), so it is non-critical.
        self.declare_parameter("wheel_separation_multiplier", 1.65)
        self.declare_parameter("ticks_per_rev", 1000.0)  # CALIBRATE (CPR*gear)

        # --- Encoder -> side mapping (indices into E1..E4 = 0..3) -------------
        self.declare_parameter("left_encoders", [0, 2])   # guess: E1,E3
        self.declare_parameter("right_encoders", [1, 3])  # guess: E2,E4
        self.declare_parameter("left_encoder_sign", 1.0)
        self.declare_parameter("right_encoder_sign", 1.0)

        # --- Motor (D,) mapping. 4 motors -> 8 PWM (R,L per motor) ------------
        # Which of the 4 motors are left/right side, and their forward sign.
        self.declare_parameter("left_motors", [0, 2])   # M1,M3 (M1=front-left)
        self.declare_parameter("right_motors", [1, 3])  # M2,M4
        self.declare_parameter("left_motor_sign", 1.0)
        self.declare_parameter("right_motor_sign", 1.0)
        # Per-motor sign, applied last. M4 (rear-right) is wired reversed: its
        # forward (_R) channel drives the wheel backward, so it needs -1 while
        # the side sign stays +1 for its partner M2.
        self.declare_parameter("motor_signs", [1.0, 1.0, 1.0, -1.0])

        # --- Velocity -> PWM -------------------------------------------------
        self.declare_parameter("max_wheel_rad_s", 7.0)   # matches URDF ±7
        self.declare_parameter("pwm_min", 40)            # static-friction floor
        self.declare_parameter("pwm_max", 255)
        self.declare_parameter("use_pid", False)         # open-loop first
        self.declare_parameter("pid_kp", 25.0)
        self.declare_parameter("pid_ki", 60.0)
        self.declare_parameter("pid_kp_ff", 30.0)        # feedforward pwm per rad/s

        # --- IMU -------------------------------------------------------------
        self.declare_parameter("gyro_bias_samples", 100)  # auto-cal at startup
        self.declare_parameter("gyro_bias", [0.0, 0.0, 0.0])  # override if >0

        # --- Safety ----------------------------------------------------------
        self.declare_parameter("cmd_timeout", 0.5)   # brake if no cmd_vel
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("publish_hz", 50.0)

        g = self.get_parameter
        self.port = g("port").value
        self.baud = int(g("baud").value)
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value
        self.imu_frame = g("imu_frame").value
        self.r = float(g("wheel_radius").value)
        self.sep = float(g("wheel_separation").value)
        self.sep_mult = float(g("wheel_separation_multiplier").value)
        self.tpr = float(g("ticks_per_rev").value)
        self.left_enc = list(g("left_encoders").value)
        self.right_enc = list(g("right_encoders").value)
        self.left_enc_sign = float(g("left_encoder_sign").value)
        self.right_enc_sign = float(g("right_encoder_sign").value)
        self.left_motors = list(g("left_motors").value)
        self.right_motors = list(g("right_motors").value)
        self.left_motor_sign = float(g("left_motor_sign").value)
        self.right_motor_sign = float(g("right_motor_sign").value)
        self.motor_signs = [float(s) for s in g("motor_signs").value]
        self.max_rad = float(g("max_wheel_rad_s").value)
        self.pwm_min = int(g("pwm_min").value)
        self.pwm_max = int(g("pwm_max").value)
        self.use_pid = bool(g("use_pid").value)
        self.kp = float(g("pid_kp").value)
        self.ki = float(g("pid_ki").value)
        self.kp_ff = float(g("pid_kp_ff").value)
        self.cmd_timeout = float(g("cmd_timeout").value)

        # --- State -----------------------------------------------------------
        self.lock = threading.Lock()
        self.enc_raw = [0, 0, 0, 0]
        self.enc_prev = None
        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.have_telem = False
        self.last_read_stamp = self.get_clock().now()

        self.x = self.y = self.th = 0.0
        self.v_left_meas = self.v_right_meas = 0.0
        self.cmd_v = self.cmd_w = 0.0
        self.last_cmd_stamp = self.get_clock().now()
        self.i_left = self.i_right = 0.0  # PID integrators

        self.gyro_bias = list(g("gyro_bias").value)
        self._bias_target = int(g("gyro_bias_samples").value)
        self._bias_acc = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._bias_done = any(abs(b) > 0 for b in self.gyro_bias)

        # --- Serial open -----------------------------------------------------
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.ser.reset_input_buffer()
        self.get_logger().info(f"ESP32 abierto en {self.port} @ {self.baud}")

        # --- ROS I/O ---------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, g("odom_topic").value, 20)
        self.imu_pub = self.create_publisher(Imu, g("imu_topic").value, 20)
        self.create_subscription(
            Twist, g("cmd_vel_topic").value, self.on_cmd_vel, 10
        )
        self.create_subscription(
            Int16MultiArray, g("arm_topic").value, self.on_arm, 10
        )

        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        self.create_timer(1.0 / float(g("publish_hz").value), self._publish)
        self.create_timer(1.0 / float(g("control_hz").value), self._control)

    # ------------------------------------------------------------------ serial
    def _read_loop(self) -> None:
        while rclpy.ok():
            try:
                raw = self.ser.readline().decode("ascii", errors="replace").strip()
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(f"serial read error: {exc}")
                continue
            if not raw:
                continue
            parts = raw.split(",")
            if len(parts) != 10:
                continue
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                continue
            with self.lock:
                self.enc_raw = [int(round(v)) for v in vals[:4]]
                self.accel = vals[4:7]
                self.gyro = vals[7:10]
                self.have_telem = True
                self.last_read_stamp = self.get_clock().now()
            if not self._bias_done:
                self._accumulate_bias(vals[7:10])

    def _accumulate_bias(self, gyro) -> None:
        for i in range(3):
            self._bias_acc[i] += gyro[i]
        self._bias_n += 1
        if self._bias_n >= self._bias_target:
            self.gyro_bias = [a / self._bias_n for a in self._bias_acc]
            self._bias_done = True
            self.get_logger().info(
                f"Gyro bias calibrado (robot quieto): {self.gyro_bias}"
            )

    def _write(self, line: str) -> None:
        with self.lock:
            try:
                self.ser.write((line + "\n").encode("ascii"))
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(f"serial write error: {exc}")

    # ------------------------------------------------------------- subscribers
    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_stamp = self.get_clock().now()

    def on_arm(self, msg: Int16MultiArray) -> None:
        angles = list(msg.data)[:6]
        if len(angles) != 6:
            self.get_logger().warn("arm: se esperaban 6 ángulos")
            return
        angles = [max(0, min(180, int(a))) for a in angles]
        self._write("A," + ",".join(str(a) for a in angles))

    # ------------------------------------------------------------------- odom
    def _side_ticks(self, enc, indices, sign):
        return sign * sum(enc[i] for i in indices) / max(1, len(indices))

    def _publish(self) -> None:
        if not rclpy.ok():
            return
        with self.lock:
            if not self.have_telem:
                return
            enc = list(self.enc_raw)
            gyro = list(self.gyro)
            accel = list(self.accel)
            stamp_ok = self.have_telem
        now = self.get_clock().now()

        left_ticks = self._side_ticks(enc, self.left_enc, self.left_enc_sign)
        right_ticks = self._side_ticks(enc, self.right_enc, self.right_enc_sign)

        if self.enc_prev is None:
            self.enc_prev = (left_ticks, right_ticks, now)
            return
        pl, pr, pt = self.enc_prev
        dt = (now - pt).nanoseconds * 1e-9
        if dt <= 0:
            return
        self.enc_prev = (left_ticks, right_ticks, now)

        # ticks -> wheel travel (m)
        m_per_tick = (2.0 * math.pi * self.r) / self.tpr
        d_left = (left_ticks - pl) * m_per_tick
        d_right = (right_ticks - pr) * m_per_tick
        self.v_left_meas = d_left / dt
        self.v_right_meas = d_right / dt

        d_center = 0.5 * (d_left + d_right)
        d_th = (d_right - d_left) / (self.sep * self.sep_mult)
        self.th += d_th
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.th)
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_th / dt
        # EKF fuses ONLY vx from here; loose pose covariance elsewhere.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.2
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.imu_frame
        # No absolute orientation from a raw accel+gyro IMU -> mark unknown.
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x = gyro[0] - self.gyro_bias[0]
        imu.angular_velocity.y = gyro[1] - self.gyro_bias[1]
        imu.angular_velocity.z = gyro[2] - self.gyro_bias[2]
        imu.linear_acceleration.x = accel[0]
        imu.linear_acceleration.y = accel[1]
        imu.linear_acceleration.z = accel[2]
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.01
        imu.linear_acceleration_covariance[0] = 0.1
        imu.linear_acceleration_covariance[4] = 0.1
        imu.linear_acceleration_covariance[8] = 0.1
        self.imu_pub.publish(imu)

    # ---------------------------------------------------------------- control
    def _rad_to_pwm(self, target_rad: float, meas_rad: float, integ_attr: str):
        """Signed target wheel speed (rad/s) -> signed PWM (-255..255)."""
        if abs(target_rad) < 1e-3:
            setattr(self, integ_attr, 0.0)
            return 0.0
        if self.use_pid:
            err = target_rad - meas_rad
            integ = getattr(self, integ_attr) + err * (1.0 / 30.0)
            integ = max(-self.max_rad, min(self.max_rad, integ))  # anti-windup
            setattr(self, integ_attr, integ)
            pwm = self.kp_ff * target_rad + self.kp * err + self.ki * integ
        else:
            # open-loop feedforward with static-friction floor
            ratio = max(-1.0, min(1.0, target_rad / self.max_rad))
            span = self.pwm_max - self.pwm_min
            pwm = math.copysign(self.pwm_min + span * abs(ratio), ratio)
        return max(-self.pwm_max, min(self.pwm_max, pwm))

    def _control(self) -> None:
        if not rclpy.ok():
            return
        now = self.get_clock().now()
        if (now - self.last_cmd_stamp).nanoseconds * 1e-9 > self.cmd_timeout:
            self.cmd_v = self.cmd_w = 0.0  # watchdog brake

        b = self.sep
        v_l = self.cmd_v - self.cmd_w * b / 2.0
        v_r = self.cmd_v + self.cmd_w * b / 2.0
        tgt_l = v_l / self.r
        tgt_r = v_r / self.r

        pwm_l = self._rad_to_pwm(tgt_l, self.v_left_meas / self.r, "i_left")
        pwm_r = self._rad_to_pwm(tgt_r, self.v_right_meas / self.r, "i_right")
        pwm_l *= self.left_motor_sign
        pwm_r *= self.right_motor_sign

        pwm = [0, 0, 0, 0]  # per-motor signed
        for i in self.left_motors:
            pwm[i] = pwm_l
        for i in self.right_motors:
            pwm[i] = pwm_r
        for i in range(4):
            pwm[i] *= self.motor_signs[i]

        fields = []
        for p in pwm:
            p = int(round(p))
            fwd = p if p > 0 else 0
            rev = -p if p < 0 else 0
            fields += [fwd, rev]  # M#_R, M#_L
        self._write("D," + ",".join(str(v) for v in fields))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._write("D,0,0,0,0,0,0,0,0")  # brake on exit
        except Exception:
            pass
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
