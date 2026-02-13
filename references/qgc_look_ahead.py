#! /usr/bin/python
# coding:utf-8

import rospy
import numpy as np
import csv
import time

from pyproj import Proj
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

# ROS custom message
from look_ahead.msg import AJK_value
from look_ahead.msg import Auto_Log
from mavlink_ajk.msg import MAV_Mission
from mavlink_ajk.msg import MAV_Modes

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply

SPACING = 0.8       # distance between lines 
x_tolerance = 0.1  # [meter]
yaw_tolerance = 40.0 # [Degree]

I_CONTROL_DIST = 0.1  # [meter], refer to cross_track_error 

# translation value
FORWARD_CONST = 1
BACKWARD_CONST = -1

# AJK
TRANSLATION_NEUTRAL = 512     # neutral value
STEERING_NEUTRAL = 512        # neutral value
RIGHT_PIVOT = 332
LEFT_PIVOT = 692
FB_OPTIMUM = 220
LR_OPTIMUM = 60

# for simulator or test vehicle
CMD_LINEAR_OPT = 1.0
CMD_ANGULAR_RIGHT = 1.0
CMD_ANGULAR_LEFT = 1.0
CMD_ANGULAR_LIMIT = 2

# frequency [Hz]
frequency = 10

# MAVLink number
MAV_CMD_NAV_WAYPOINT = 16
ARDUPILOT_AUTO_BASE = 217
ARDUPILOT_AUTO_CUSTOM = 10

class look_ahead():
    def __init__(self):
        self.waypoint_x = []
        self.waypoint_y = []
        self.last_waypoint_x = 0
        self.last_waypoint_y = 0
        self.waypoint_seq = []
        self.waypoint_total_seq = 0
        self.x = 0
        self.y = 0
        self.q = np.empty(4)
        self.yaw = np.pi/2
        self.pre_steering_ang = 0

        # mav_modes
        self.mission_start = False
        self.base_mode = 0
        self.custom_mode = 0

        rospy.init_node('look_ahead_following')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function, receive /odom mesage
        rospy.Subscriber('/gnss_odom', Odometry, self.odom_callback, queue_size = 1)
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.truth_callback)
        rospy.Subscriber('/mav/mission', MAV_Mission, self.load_waypoint)
        rospy.Subscriber('/mav/modes', MAV_Modes, self.mav_modes)
        self.ajk_pub = rospy.Publisher('/ajk_auto', AJK_value, queue_size = 1)
        self.ajk_value = AJK_value()
        self.auto_log_pub = rospy.Publisher('/auto_log', Auto_Log, queue_size = 1)
        self.auto_log = Auto_Log()
        self.cmdvel_pub = rospy.Publisher('/sim_ajk/diff_drive_controller/cmd_vel', Twist, queue_size = 1)
        self.cmdvel = Twist()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # vehicle's quaternion data in /odom (odometry of ROS message)
        self.q[0] = msg.pose.pose.orientation.x
        self.q[1] = msg.pose.pose.orientation.y
        self.q[2] = msg.pose.pose.orientation.z
        self.q[3] = msg.pose.pose.orientation.w

    # truth position of simulator
    #def truth_callback(self, msg):
    #    for i, name in enumerate(msg.name):
    #        if name == "sim_ajk":
    #            self.x = msg.pose[i].position.x
    #            self.y = msg.pose[i].position.y
    #            # vehicle's quaternion data in /odom (odometry of ROS message)
    #            self.q[0] = msg.pose[i].orientation.x
    #            self.q[1] = msg.pose[i].orientation.y
    #            self.q[2] = msg.pose[i].orientation.z
    #            self.q[3] = msg.pose[i].orientation.w

    # load waypoint list
    def load_waypoint(self, msg):
        # UTM coordinate calculation
        if msg.command == MAV_CMD_NAV_WAYPOINT:
            utmzone = int((msg.longitude + 180)/6) +1   # If you are on the specific location, can't be calculated. 
            convertor = Proj(proj='utm', zone=utmzone, ellps='WGS84')
            x, y = convertor(msg.longitude, msg.latitude)
            self.last_waypoint_x = x
            self.last_waypoint_y = y
        else:
        # if not the MAV_CMD_NAV_WAYPOINT, previous waypoint will be added 
            x = self.last_waypoint_x
            y = self.last_waypoint_y
        #print x, y
        #print msg.longitude, msg.latitude

        # if msg.seq is 0, then reset variables and receive the new mission's
        if msg.seq == 0:
            self.waypoint_x = []
            self.waypoint_y = []
            self.waypoint_seq = []
        
        self.waypoint_seq.append(msg.seq)
        self.waypoint_total_seq = msg.total_seq
        self.waypoint_x.append(x)
        self.waypoint_y.append(y) 
        #print self.waypoint_total_seq, self.waypoint_seq, self.waypoint_x, self.waypoint_y

    def mav_modes(self, msg):
        self.mission_start = msg.mission_start
        self.base_mode = msg.base_mode
        self.custom_mode = msg.custom_mode

    def cmdvel_publisher(self, steering_ang, translation, pid):
        if abs(steering_ang) > yaw_tolerance:
            if steering_ang >= 0:
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = CMD_ANGULAR_LEFT
            else:
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = CMD_ANGULAR_RIGHT
        else:
            self.cmdvel.linear.x = CMD_LINEAR_OPT*translation
            self.cmdvel.angular.z = pid

        # Angular limit
        if self.cmdvel.angular.z > CMD_ANGULAR_LIMIT:
            self.cmdvel.angular.z = CMD_ANGULAR_LIMIT
        elif self.cmdvel.angular.z < -CMD_ANGULAR_LIMIT:
            self.cmdvel.angular.z = -CMD_ANGULAR_LIMIT
        self.cmdvel_pub.publish(self.cmdvel)

    def shutdown(self):
        print "shutdown"

    def loop(self):
        rr = rospy.Rate(frequency)
        seq = 1
        KP = 0
        KI = 0
        look_ahead_dist = 0
        while not rospy.is_shutdown():
            # mission checker
            if self.waypoint_total_seq != len(self.waypoint_seq) or self.waypoint_total_seq == 0:
                seq = 1
                rospy.loginfo("mission_checker")
                time.sleep(1)
                continue

            # mission_start checker(origin from MAV_CMD_MISSION_START)
            if self.mission_start != True or self.base_mode != ARDUPILOT_AUTO_BASE \
            or self.custom_mode != ARDUPILOT_AUTO_CUSTOM:
                rospy.loginfo("mission_start_checker")
                time.sleep(1)
                continue

            # if a specific variable is exists, the proceeds 
            try:
                own_x = self.x
                own_y = self.y
                front_q = self.q
            except AttributeError:
                continue

            # get the parameters of look-ahead control
            KP = rospy.get_param("/mavlink_ajk/Kp")
            KI = rospy.get_param("/mavlink_ajk/Ki")
            look_ahead_dist = rospy.get_param("/mavlink_ajk/look_ahead")

            # waypoint with xy coordinate origin adjust
            if seq == 0:
                wp_x_adj = self.waypoint_x[seq] - own_x
                wp_y_adj = self.waypoint_y[seq] - own_y
                own_x_adj = 0
                own_y_adj = 0
            else:
                wp_x_adj = self.waypoint_x[seq] - self.waypoint_x[seq-1]
                wp_y_adj = self.waypoint_y[seq] - self.waypoint_y[seq-1]
                own_x_adj = own_x - self.waypoint_x[seq-1]
                own_y_adj = own_y - self.waypoint_y[seq-1]

            # coordinate transformation of waypoint
            tf_angle = np.arctan2(wp_y_adj, wp_x_adj)
            wp_x_tf = wp_x_adj*np.cos(-tf_angle) - wp_y_adj*np.sin(-tf_angle)
            wp_y_tf = wp_x_adj*np.sin(-tf_angle) + wp_y_adj*np.cos(-tf_angle)

            # coordinate transformation of own position
            own_x_tf = own_x_adj*np.cos(-tf_angle) - own_y_adj*np.sin(-tf_angle)
            own_y_tf = own_x_adj*np.sin(-tf_angle) + own_y_adj*np.cos(-tf_angle)


            # coordinate transformation of own position
            tf_q = quaternion_from_euler(0, 0, tf_angle)
            front_q_tf = quaternion_multiply((front_q[0], front_q[1], front_q[2], front_q[3]), 
                                             (   tf_q[0],    tf_q[1],    tf_q[2],   -tf_q[3]))

            # inverted
            rear_q_tf = np.empty(4)
            rear_q_tf[0] = front_q_tf[0]
            rear_q_tf[1] = front_q_tf[1]
            rear_q_tf[2] = front_q_tf[3]
            rear_q_tf[3] = -front_q_tf[2]

            # calculate the target-angle(bearing) using look-ahead distance
            bearing = np.arctan2(-own_y_tf, look_ahead_dist)
            bearing_q = quaternion_from_euler(0, 0, bearing)

            # calculate the minimal yaw error, and decide the forward or backward
            front_steering_q = quaternion_multiply(( bearing_q[0],  bearing_q[1],  bearing_q[2],  bearing_q[3]),
                                                   (front_q_tf[0], front_q_tf[1], front_q_tf[2], -front_q_tf[3]))
            rear_steering_q = quaternion_multiply((bearing_q[0], bearing_q[1], bearing_q[2], bearing_q[3]),
                                                  (rear_q_tf[0], rear_q_tf[1], rear_q_tf[2], -rear_q_tf[3]))

            front_steering_ang = euler_from_quaternion(front_steering_q)[2]/np.pi *180
            rear_steering_ang = euler_from_quaternion(rear_steering_q)[2]/np.pi *180

            if abs(front_steering_ang) >= abs(rear_steering_ang):
                steering_ang = rear_steering_ang
                translation = BACKWARD_CONST
            elif abs(front_steering_ang) < abs(rear_steering_ang):
                steering_ang = front_steering_ang
                translation = FORWARD_CONST

            #print steering_ang/np.pi*180.0

            # calculate the steering_value
            p = KP *steering_ang
            i = KI *own_y_tf
            
            pid_value = p
            if abs(own_y_tf) < I_CONTROL_DIST:
                pid_value = p - i

            ajk_steering = STEERING_NEUTRAL +LR_OPTIMUM *pid_value
            if translation < 0:
                ajk_steering = STEERING_NEUTRAL -LR_OPTIMUM *pid_value 
            ajk_translation = TRANSLATION_NEUTRAL +FB_OPTIMUM *translation

            # Restriction of ajk_steering
            if ajk_steering > TRANSLATION_NEUTRAL + LR_OPTIMUM:
                ajk_steering = TRANSLATION_NEUTRAL + LR_OPTIMUM
            elif ajk_steering < TRANSLATION_NEUTRAL - LR_OPTIMUM:
                ajk_steering = TRANSLATION_NEUTRAL - LR_OPTIMUM

            # If the yaw error is large, pivot turn.
            if abs(steering_ang) > yaw_tolerance:
                if steering_ang >= 0:
                    self.ajk_value.stamp = rospy.Time.now()
                    self.ajk_value.translation = TRANSLATION_NEUTRAL
                    self.ajk_value.steering = LEFT_PIVOT
                else:
                    self.ajk_value.stamp = rospy.Time.now()
                    self.ajk_value.translation = TRANSLATION_NEUTRAL
                    self.ajk_value.steering = RIGHT_PIVOT                    
            else:
                self.ajk_value.stamp = rospy.Time.now()
                self.ajk_value.translation = ajk_translation
                self.ajk_value.steering = ajk_steering
            self.ajk_pub.publish(self.ajk_value)

            # for simulator
            self.cmdvel_publisher(steering_ang, translation, pid_value)

            # publish autonomous log
            self.auto_log.stamp = rospy.Time.now()
            self.auto_log.waypoint_seq = seq
            self.auto_log.waypoint_x = self.waypoint_x[seq]
            self.auto_log.waypoint_y = self.waypoint_y[seq]
            self.auto_log.own_x = own_x
            self.auto_log.own_y = own_y
            self.auto_log.own_yaw = euler_from_quaternion(front_q)[2]/np.pi *180

            self.auto_log.tf_waypoint_x = wp_x_tf
            self.auto_log.tf_waypoint_y = wp_y_tf
            self.auto_log.tf_own_x = own_x_tf
            self.auto_log.tf_own_y = own_y_tf
            self.auto_log.cross_track_error = -own_y_tf

            self.auto_log.Kp = KP
            self.auto_log.Ki = KI
            self.auto_log.look_ahead_dist = look_ahead_dist

            self.auto_log.p = p
            self.auto_log.i = i
            self.auto_log.steering_ang = steering_ang
            self.auto_log.translation = self.ajk_value.translation
            self.auto_log.steering = self.ajk_value.steering
            self.auto_log_pub.publish(self.auto_log)

            # when reaching the look-ahead distance, read the next waypoint.
            if (wp_x_tf - own_x_tf) < x_tolerance:
                pre_wp_x = self.waypoint_x[seq]
                pre_wp_y = self.waypoint_y[seq]
                seq = seq + 1
                try:
                    a = np.array([pre_wp_x, pre_wp_y])
                    b = np.array([self.waypoint_x[seq], self.waypoint_y[seq]])

                    if np.linalg.norm(a-b) < SPACING:
                        seq = seq + 1
                except IndexError:
                    pass

            if seq >= len(self.waypoint_x):
                self.ajk_value.stamp = rospy.Time.now()
                self.ajk_value.translation = TRANSLATION_NEUTRAL
                self.ajk_value.steering = STEERING_NEUTRAL
                self.ajk_pub.publish(self.ajk_value)

                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = 0
                self.cmdvel_pub.publish(self.cmdvel)
                seq = 1
                print "mission_end"
                break
            #print
            #time.sleep(1/frequency)
            rr.sleep()
  
if __name__ == '__main__':
    l = look_ahead()
    l.loop()
