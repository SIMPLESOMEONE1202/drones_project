from email.mime import base
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header

from drone_msgs.msg import DroneState


def quaternion_to_euler(x, y, z, w):
    """Standard quaternion -> roll, pitch, yaw (radians), no external deps."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def normalize_angle(angle):
    """Wrap an angle (radians) into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    def __init__(self, kp, ki, kd, out_min=-1e9, out_max=1e9, i_limit=1e9, d_filter_alpha=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_limit = i_limit
        self.d_filter_alpha = d_filter_alpha  # 0 = no filtering, 1 = fully smoothed
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._filtered_derivative = 0.0

    def update(self, error, now_sec):
        if self._prev_time is None:
            dt = 0.01
        else:
            dt = max(1e-3, now_sec - self._prev_time)   # raised floor from 1e-4 -> 1e-3
        self._prev_time = now_sec

        self._integral += error * dt
        self._integral = max(-self.i_limit, min(self.i_limit, self._integral))

        raw_derivative = (error - self._prev_error) / dt
        self._prev_error = error

        # Low-pass filter the derivative to kill high-frequency kick
        self._filtered_derivative = (
            self.d_filter_alpha * raw_derivative
            + (1 - self.d_filter_alpha) * self._filtered_derivative
        )

        output = self.kp * error + self.ki * self._integral + self.kd * self._filtered_derivative
        return max(self.out_min, min(self.out_max, output))


class PidControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        self.declare_parameter('drone_id', 'drone_1')
        self.drone_id = self.get_parameter('drone_id').value
        self.target_altitude = 2.0     # metres
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw_rate = 0.0

        # Physical constants matching the xacro's multicopter_motor_model plugin
        self.motor_constant = 8.54858e-06
        self.max_rot_velocity = 838.0
        total_mass = 1.59  # base_link + imu_link + 4 rotors, from quadrotor_macro.xacro
        gravity = 9.81
        thrust_per_rotor = (total_mass * gravity) / 4.0
        self.hover_speed = math.sqrt(thrust_per_rotor / self.motor_constant)
        self.get_logger().info(f'Computed hover speed: {self.hover_speed:.2f} rad/s')

        # PID gains — starting points, expect to tune after first flight test
        self.pid_alt = PID(
            kp=8.0,
            ki=1.0,
            kd=6.0,
            out_min=-300,
            out_max=300,
            i_limit=50
        )
        self.pid_roll = PID(
            kp=12.0,
            ki=0.0,
            kd=1.5,
            out_min=-60.0,
            out_max=60.0,
            i_limit=10.0
        )

        self.pid_pitch = PID(
            kp=12.0,
            ki=0.0,
            kd=1.5,
            out_min=-60.0,
            out_max=60.0,
            i_limit=10.0
        )
        self.pid_yaw = PID(
            kp=8.0,
            ki=0.0,
            kd=1.0,
            out_min=-30.0,
            out_max=30.0,
            i_limit=5.0
        )
        # Velocity-hold PIDs — output becomes the target angle for the attitude PIDs above,
        # so the drone actively resists horizontal drift instead of just holding attitude=0
        self.pid_vx = PID(kp=0.06, ki=0.0, kd=0.02, out_min=-0.15, out_max=0.15, i_limit=0.5)
        self.pid_vy = PID(kp=0.06, ki=0.0, kd=0.02, out_min=-0.15, out_max=0.15, i_limit=0.5)

        self._latest_imu = None
        self._latest_odom = None

        self.imu_sub = self.create_subscription(
            Imu, f'/{self.drone_id}/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, f'/{self.drone_id}/odom', self.odom_callback, 10)

        self.motor_pub = self.create_publisher(
            Actuators, f'/{self.drone_id}/command/motor_speed', 10)
        self.state_pub = self.create_publisher(
            DroneState, f'/{self.drone_id}/state', 10)

        self.control_period = 0.01  # 100 Hz
        self.timer = self.create_timer(self.control_period, self.control_loop)

    def imu_callback(self, msg):
        self._latest_imu = msg

    def odom_callback(self, msg):
        self._latest_odom = msg

    def control_loop(self):
        if self._latest_imu is None or self._latest_odom is None:
            return  # no sensor data yet — nothing to control on

        now_sec = self.get_clock().now().nanoseconds / 1e9

        q = self._latest_imu.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        yaw_rate = self._latest_imu.angular_velocity.z

        z = self._latest_odom.pose.pose.position.z
        vz = self._latest_odom.twist.twist.linear.z

        alt_error = self.target_altitude - z

        # Vertical velocity
        vz = self._latest_odom.twist.twist.linear.z

        # Desired vertical velocity from altitude error
        # Far from target -> faster climb/descent
        # Near target -> slow down
        max_vz = 0.4

        desired_vz = max(-max_vz, min(max_vz, 0.4 * alt_error))

        # Vertical velocity error
        vz_error = desired_vz - vz

        # Position + velocity control
        alt_out = (
            5.0 * alt_error
            + 8.0 * vz_error
        )

        # Limit altitude correction
        alt_out = max(
            self.pid_alt.out_min,
            min(self.pid_alt.out_max, alt_out)
        )

        vx = self._latest_odom.twist.twist.linear.x
        vy = self._latest_odom.twist.twist.linear.y

        # Outer loop: velocity error -> desired tilt angle (small angles only, clamped above)
        target_pitch_dynamic = self.target_pitch
        target_roll_dynamic = self.target_roll

        roll_error = normalize_angle(target_roll_dynamic - roll)
        roll_out = self.pid_roll.update(roll_error, now_sec)

        pitch_error = normalize_angle(target_pitch_dynamic - pitch)
        pitch_out = self.pid_pitch.update(pitch_error, now_sec)
        

        yaw_error = self.target_yaw_rate - yaw_rate
        yaw_out = self.pid_yaw.update(yaw_error, now_sec)

        # --- Motor mixing (X configuration) ---
        # rotor0: front-left (ccw)   rotor1: front-right (cw)
        # rotor2: back-right (ccw)   rotor3: back-left (cw)
        base = self.hover_speed + alt_out
        m0 = base + roll_out - pitch_out - yaw_out
        m1 = base - roll_out - pitch_out + yaw_out
        m2 = base - roll_out + pitch_out - yaw_out
        m3 = base + roll_out + pitch_out + yaw_out

        self.get_logger().info(
            f'z={z:.3f} vz={vz:.3f} '
            f'alt_err={alt_error:.3f} alt_out={alt_out:.3f} '
            f'base={base:.2f} '
            f'vx={vx:.3f} vy={vy:.3f} '
            f'target_pitch={target_pitch_dynamic:.3f} pitch={pitch:.3f} pitch_out={pitch_out:.3f} '
            f'target_roll={target_roll_dynamic:.3f} roll={roll:.3f} roll_out={roll_out:.3f}'
        )

        motor_speeds = [
            max(0.0, min(self.max_rot_velocity, m))
            for m in (m0, m1, m2, m3)
        ]

        actuators_msg = Actuators()
        actuators_msg.velocity = motor_speeds
        self.motor_pub.publish(actuators_msg)

        # --- Telemetry ---
        state_msg = DroneState()
        state_msg.header = Header()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.drone_id = self.drone_id
        state_msg.x = self._latest_odom.pose.pose.position.x
        state_msg.y = self._latest_odom.pose.pose.position.y
        state_msg.z = z
        state_msg.roll = roll
        state_msg.pitch = pitch
        state_msg.yaw = yaw
        state_msg.vx = self._latest_odom.twist.twist.linear.x
        state_msg.vy = self._latest_odom.twist.twist.linear.y
        state_msg.vz = vz
        state_msg.battery_percent = 100.0   # placeholder — battery model comes in Step 6
        state_msg.battery_voltage = 12.6    # placeholder
        state_msg.mission_state = 'PATROL'
        state_msg.comms_active = True
        state_msg.known_peers = 0
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PidControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()