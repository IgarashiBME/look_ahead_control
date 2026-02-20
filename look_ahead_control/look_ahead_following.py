#!/usr/bin/env python3
"""Look-Ahead path following controller node (ROS2).

Ported from references/qgc_look_ahead.py (ROS1).
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from pyproj import Proj

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, UInt16MultiArray
from geometry_msgs.msg import Twist

from bme_common_msgs.msg import AutoLog
from bme_common_msgs.msg import GnssSolution
from bme_common_msgs.msg import MavModes

from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_multiply

# translation value
FORWARD_CONST = 1
BACKWARD_CONST = -1

# frequency [Hz]
FREQUENCY = 10

# MAVLink number
MAV_CMD_NAV_WAYPOINT = 16
ARDUPILOT_AUTO_BASE = 217
ARDUPILOT_AUTO_CUSTOM = 10


class LookAheadFollowing(Node):
    def __init__(self):
        super().__init__('look_ahead_following')

        self.waypoint_x = []
        self.waypoint_y = []
        self.last_waypoint_x = 0.0
        self.last_waypoint_y = 0.0
        self.waypoint_seq = []
        self.waypoint_total_seq = 0
        self.x = 0.0
        self.y = 0.0
        self.q = np.empty(4)
        self.yaw = np.pi / 2
        self.pre_steering_ang = 0.0

        # mav_modes
        self.mission_start = False
        self.base_mode = 0
        self.custom_mode = 0

        # Declare parameters with defaults
        self.declare_parameter('Kp', 0.0)
        self.declare_parameter('Kcte', 0.0)
        self.declare_parameter('look_ahead', 0.0)
        self.declare_parameter('pivot_threshold', 40.0)
        self.declare_parameter('cte_threshold', 0.1)
        self.declare_parameter('wp_arrival_dist', 0.1)
        self.declare_parameter('wp_skip_dist', 0.8)
        self.declare_parameter('throttle_scale', 0.5)
        self.declare_parameter('pivot_scale', 0.5)
        self.declare_parameter('driver_mix', 0.0)
        self.declare_parameter('pwm_center', 1500.0)
        self.declare_parameter('pwm_range', 500.0)
        self.declare_parameter('pwm_min', 1000.0)
        self.declare_parameter('pwm_max', 2000.0)
        self.declare_parameter('odom_source', 'odom')

        # Subscribers — select odometry source by parameter
        odom_source = self.get_parameter(
            'odom_source').get_parameter_value().string_value

        if odom_source == 'gnss':
            self.create_subscription(
                GnssSolution, '/gnss', self.gnss_callback,
                QoSProfile(depth=1))
            self.get_logger().info("odom_source: /gnss (GnssSolution)")
        else:
            self.create_subscription(
                Odometry, '/gnss_odom', self.odom_callback,
                QoSProfile(depth=1))
            self.get_logger().info("odom_source: /gnss_odom (Odometry)")

        # /mav/mission: Float64MultiArray [seq, total_seq, command, lat, lon]
        self.create_subscription(
            Float64MultiArray, '/mav/mission', self.load_waypoint,
            QoSProfile(depth=1000))

        self.create_subscription(
            MavModes, '/mav/modes', self.mav_modes_callback,
            QoSProfile(depth=1))

        # Publishers
        self.auto_log_pub = self.create_publisher(
            AutoLog, '/auto_log', QoSProfile(depth=1))

        self.rc_pwm_pub = self.create_publisher(
            UInt16MultiArray, '/rc_pwm', QoSProfile(depth=1))

        self.cmdvel_pub = self.create_publisher(
            Twist, '/cmd_vel', QoSProfile(depth=1))
        self.cmdvel = Twist()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # vehicle's quaternion data in /odom (odometry of ROS message)
        self.q[0] = msg.pose.pose.orientation.x
        self.q[1] = msg.pose.pose.orientation.y
        self.q[2] = msg.pose.pose.orientation.z
        self.q[3] = msg.pose.pose.orientation.w

    def gnss_callback(self, msg):
        self.x = msg.utm_easting
        self.y = msg.utm_northing

        # heading_deg (ENU: East=0, CCW positive) → quaternion
        heading_rad = msg.heading_deg * np.pi / 180.0
        self.q = quaternion_from_euler(0.0, 0.0, heading_rad)

    def load_waypoint(self, msg):
        """Load waypoint from Float64MultiArray [seq, total_seq, command, lat, lon]."""
        if len(msg.data) < 5:
            return

        seq = int(msg.data[0])
        total_seq = int(msg.data[1])
        command = int(msg.data[2])
        latitude = msg.data[3]
        longitude = msg.data[4]

        # UTM coordinate calculation
        if command == MAV_CMD_NAV_WAYPOINT:
            utmzone = int((longitude + 180) / 6) + 1
            convertor = Proj(proj='utm', zone=utmzone, ellps='WGS84')
            x, y = convertor(longitude, latitude)
            self.last_waypoint_x = x
            self.last_waypoint_y = y
        else:
            # if not MAV_CMD_NAV_WAYPOINT, use previous waypoint
            x = self.last_waypoint_x
            y = self.last_waypoint_y

        # if seq is 0, reset variables and receive the new mission
        if seq == 0:
            self.waypoint_x = []
            self.waypoint_y = []
            self.waypoint_seq = []

        self.waypoint_seq.append(seq)
        self.waypoint_total_seq = total_seq
        self.waypoint_x.append(x)
        self.waypoint_y.append(y)

    def mav_modes_callback(self, msg):
        self.mission_start = msg.mission_start
        self.base_mode = msg.base_mode
        self.custom_mode = msg.custom_mode

    def control_publish(self, steering_ang, translation, pid):
        """Compute RC PWM (primary) and derive cmd_vel."""
        throttle_scale = self.get_parameter(
            'throttle_scale').get_parameter_value().double_value
        pivot_scale = self.get_parameter(
            'pivot_scale').get_parameter_value().double_value
        pivot_threshold = self.get_parameter(
            'pivot_threshold').get_parameter_value().double_value
        driver_mix = self.get_parameter(
            'driver_mix').get_parameter_value().double_value

        # Compute throttle and steering
        if abs(steering_ang) > pivot_threshold:
            # Pivot turn
            if steering_ang >= 0:
                throttle = 0.0
                steering = pivot_scale
            else:
                throttle = 0.0
                steering = -pivot_scale
        else:
            # Normal drive
            throttle = throttle_scale * translation
            steering = pid

        # Mode-dependent channel assignment
        if driver_mix >= 0.5:
            # Passthrough: driver handles mixing
            # ch1=throttle, ch2=steering
            ch1 = max(-1.0, min(1.0, throttle))
            ch2 = max(-1.0, min(1.0, steering))
        else:
            # Differential: node mixes into left/right
            # ch1=left, ch2=right
            ch1 = max(-1.0, min(1.0, throttle - steering))
            ch2 = max(-1.0, min(1.0, throttle + steering))

        # PWM parameters
        pwm_center = self.get_parameter(
            'pwm_center').get_parameter_value().double_value
        pwm_range = self.get_parameter(
            'pwm_range').get_parameter_value().double_value
        pwm_min = self.get_parameter(
            'pwm_min').get_parameter_value().double_value
        pwm_max = self.get_parameter(
            'pwm_max').get_parameter_value().double_value

        # Map to PWM
        ch1_pwm = int(pwm_center + ch1 * pwm_range)
        ch2_pwm = int(pwm_center + ch2 * pwm_range)

        # Safety clamp
        ch1_pwm = max(int(pwm_min), min(int(pwm_max), ch1_pwm))
        ch2_pwm = max(int(pwm_min), min(int(pwm_max), ch2_pwm))

        # Publish RC PWM (primary output)
        rc_msg = UInt16MultiArray()
        rc_msg.data = [ch1_pwm, ch2_pwm]
        self.rc_pwm_pub.publish(rc_msg)

        # Derive cmd_vel from PWM (secondary output)
        if pwm_range != 0:
            ch1_n = (ch1_pwm - pwm_center) / pwm_range
            ch2_n = (ch2_pwm - pwm_center) / pwm_range
        else:
            ch1_n = 0.0
            ch2_n = 0.0

        if driver_mix >= 0.5:
            # Passthrough: ch1=throttle, ch2=steering
            self.cmdvel.linear.x = ch1_n
            self.cmdvel.angular.z = ch2_n
        else:
            # Differential: ch1=left, ch2=right
            self.cmdvel.linear.x = (ch1_n + ch2_n) / 2.0
            self.cmdvel.angular.z = (ch2_n - ch1_n) / 2.0

        self.cmdvel_pub.publish(self.cmdvel)

    def loop(self):
        rate = self.create_rate(FREQUENCY)
        seq = 1
        KP = 0.0
        KCTE = 0.0
        look_ahead_dist = 0.0

        while rclpy.ok():
            # mission checker
            if self.waypoint_total_seq != len(self.waypoint_seq) \
                    or self.waypoint_total_seq == 0:
                seq = 1
                self.get_logger().info(
                    "mission_checker", throttle_duration_sec=1.0)
                rate.sleep()
                continue

            # mission_start checker (origin from MAV_CMD_MISSION_START)
            if not self.mission_start \
                    or self.base_mode != ARDUPILOT_AUTO_BASE \
                    or self.custom_mode != ARDUPILOT_AUTO_CUSTOM:
                self.get_logger().info(
                    "mission_start_checker", throttle_duration_sec=1.0)
                rate.sleep()
                continue

            # if a specific variable exists, proceed
            try:
                own_x = self.x
                own_y = self.y
                front_q = self.q.copy()
            except AttributeError:
                rate.sleep()
                continue

            # get the parameters of look-ahead control
            KP = self.get_parameter('Kp').get_parameter_value().double_value
            KCTE = self.get_parameter('Kcte').get_parameter_value().double_value
            look_ahead_dist = self.get_parameter(
                'look_ahead').get_parameter_value().double_value
            cte_threshold = self.get_parameter(
                'cte_threshold').get_parameter_value().double_value
            wp_arrival_dist = self.get_parameter(
                'wp_arrival_dist').get_parameter_value().double_value
            wp_skip_dist = self.get_parameter(
                'wp_skip_dist').get_parameter_value().double_value

            # waypoint with xy coordinate origin adjust
            if seq == 0:
                wp_x_adj = self.waypoint_x[seq] - own_x
                wp_y_adj = self.waypoint_y[seq] - own_y
                own_x_adj = 0.0
                own_y_adj = 0.0
            else:
                wp_x_adj = self.waypoint_x[seq] - self.waypoint_x[seq - 1]
                wp_y_adj = self.waypoint_y[seq] - self.waypoint_y[seq - 1]
                own_x_adj = own_x - self.waypoint_x[seq - 1]
                own_y_adj = own_y - self.waypoint_y[seq - 1]

            # coordinate transformation of waypoint
            tf_angle = np.arctan2(wp_y_adj, wp_x_adj)
            wp_x_tf = (wp_x_adj * np.cos(-tf_angle)
                       - wp_y_adj * np.sin(-tf_angle))
            wp_y_tf = (wp_x_adj * np.sin(-tf_angle)
                       + wp_y_adj * np.cos(-tf_angle))

            # coordinate transformation of own position
            own_x_tf = (own_x_adj * np.cos(-tf_angle)
                        - own_y_adj * np.sin(-tf_angle))
            own_y_tf = (own_x_adj * np.sin(-tf_angle)
                        + own_y_adj * np.cos(-tf_angle))

            # coordinate transformation of own orientation
            tf_q = quaternion_from_euler(0, 0, tf_angle)
            front_q_tf = quaternion_multiply(
                (front_q[0], front_q[1], front_q[2], front_q[3]),
                (tf_q[0], tf_q[1], tf_q[2], -tf_q[3]))

            # inverted
            rear_q_tf = np.empty(4)
            rear_q_tf[0] = front_q_tf[0]
            rear_q_tf[1] = front_q_tf[1]
            rear_q_tf[2] = front_q_tf[3]
            rear_q_tf[3] = -front_q_tf[2]

            # calculate the target-angle (bearing) using look-ahead distance
            bearing = np.arctan2(-own_y_tf, look_ahead_dist)
            bearing_q = quaternion_from_euler(0, 0, bearing)

            # calculate the minimal yaw error, and decide forward or backward
            front_steering_q = quaternion_multiply(
                (bearing_q[0], bearing_q[1], bearing_q[2], bearing_q[3]),
                (front_q_tf[0], front_q_tf[1], front_q_tf[2],
                 -front_q_tf[3]))
            rear_steering_q = quaternion_multiply(
                (bearing_q[0], bearing_q[1], bearing_q[2], bearing_q[3]),
                (rear_q_tf[0], rear_q_tf[1], rear_q_tf[2],
                 -rear_q_tf[3]))

            front_steering_ang = (
                euler_from_quaternion(front_steering_q)[2] / np.pi * 180)
            rear_steering_ang = (
                euler_from_quaternion(rear_steering_q)[2] / np.pi * 180)

            if abs(front_steering_ang) >= abs(rear_steering_ang):
                steering_ang = rear_steering_ang
                translation = BACKWARD_CONST
            elif abs(front_steering_ang) < abs(rear_steering_ang):
                steering_ang = front_steering_ang
                translation = FORWARD_CONST

            # calculate the steering value
            p = KP * steering_ang
            cte = KCTE * own_y_tf

            pid_value = p
            if abs(own_y_tf) < cte_threshold:
                pid_value = p - cte

            # publish rc_pwm (primary) and cmd_vel (derived)
            self.control_publish(steering_ang, translation, pid_value)

            # publish auto_log
            auto_log_msg = AutoLog()
            auto_log_msg.stamp = self.get_clock().now().to_msg()
            auto_log_msg.waypoint_seq = seq
            auto_log_msg.waypoint_start_x = \
                self.waypoint_x[seq - 1] if seq > 0 else 0.0
            auto_log_msg.waypoint_start_y = \
                self.waypoint_y[seq - 1] if seq > 0 else 0.0
            auto_log_msg.waypoint_end_x = self.waypoint_x[seq]
            auto_log_msg.waypoint_end_y = self.waypoint_y[seq]
            auto_log_msg.own_x = own_x
            auto_log_msg.own_y = own_y
            auto_log_msg.own_yaw = float(
                euler_from_quaternion(front_q)[2] / np.pi * 180)
            auto_log_msg.tf_waypoint_x = float(wp_x_tf)
            auto_log_msg.tf_waypoint_y = float(wp_y_tf)
            auto_log_msg.tf_own_x = float(own_x_tf)
            auto_log_msg.tf_own_y = float(own_y_tf)
            auto_log_msg.cross_track_error = float(-own_y_tf)
            auto_log_msg.kp = KP
            auto_log_msg.kcte = KCTE
            auto_log_msg.look_ahead_dist = look_ahead_dist
            auto_log_msg.p = float(p)
            auto_log_msg.cte = float(cte)
            auto_log_msg.steering_ang = float(steering_ang)
            auto_log_msg.linear_x = self.cmdvel.linear.x
            auto_log_msg.angular_z = self.cmdvel.angular.z
            self.auto_log_pub.publish(auto_log_msg)

            # when reaching the look-ahead distance, read the next waypoint
            if (wp_x_tf - own_x_tf) < wp_arrival_dist:
                pre_wp_x = self.waypoint_x[seq]
                pre_wp_y = self.waypoint_y[seq]
                seq = seq + 1
                try:
                    a = np.array([pre_wp_x, pre_wp_y])
                    b = np.array([self.waypoint_x[seq],
                                  self.waypoint_y[seq]])

                    if np.linalg.norm(a - b) < wp_skip_dist:
                        seq = seq + 1
                except IndexError:
                    pass

            if seq >= len(self.waypoint_x):
                # Stop: publish neutral PWM
                pwm_center = int(self.get_parameter(
                    'pwm_center').get_parameter_value().double_value)
                rc_stop = UInt16MultiArray()
                rc_stop.data = [pwm_center, pwm_center]
                self.rc_pwm_pub.publish(rc_stop)
                self.cmdvel.linear.x = 0.0
                self.cmdvel.angular.z = 0.0
                self.cmdvel_pub.publish(self.cmdvel)
                seq = 1
                self.get_logger().info("mission_end")
                break

            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = LookAheadFollowing()

    # Spin in a separate thread to process subscription callbacks
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("shutdown")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
