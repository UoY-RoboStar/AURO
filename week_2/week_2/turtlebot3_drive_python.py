#
# Copyright (c) 2024 University of York and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation
#

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion

import math
from enum import Enum

LINEAR_VELOCITY  = 0.3
ANGULAR_VELOCITY = 1.5

CENTER = 0
LEFT = 1
RIGHT = 2

# Enumeration to keep track of current state
class State(Enum):
    GET_TB3_DIRECTION = 0
    TB3_DRIVE_FORWARD = 1
    TB3_RIGHT_TURN = 2
    TB3_LEFT_TURN = 3

class Turtlebot3DrivePython(Node):

    def __init__(self):
        super().__init__('turtlebot3_drive_python')

        ##########################
        ## Initialise variables ##
        ##########################

        self.scan_data_ = [0.0] * 3

        self.robot_pose_ = 0.0
        self.prev_robot_pose_ = 0.0

        self.turtlebot3_state_num = State.GET_TB3_DIRECTION

        ###############################################
        ## Initialise ROS publishers and subscribers ##
        ###############################################

        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST,
                         depth = 10)
        
        # Initialise publishers
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', qos)

        # Initialise subscribers
        self.scan_sub_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.scan_sub_  # prevent unused variable warning

        self.odom_sub_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos)
        self.odom_sub_  # prevent unused variable warning

        ###########################
        ## Initialise ROS timers ##
        ###########################

        self.update_timer_ = self.create_timer(0.01, self.update_callback)

        self.get_logger().info("Turtlebot3 simulation node has been initialised")

    ############################################
    ## Callback functions for ROS subscribers ##
    ############################################

    def odom_callback(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.robot_pose_ = yaw

        self.get_logger().info(f"robot_pose_: {self.robot_pose_}")


    def scan_callback(self, msg):
        scan_angle = [0, 30, 330]

        for num in range(0, 3):
            if math.isinf(msg.ranges[scan_angle[num]]):
                self.scan_data_[num] = msg.range_max
            else:
                self.scan_data_[num] = msg.ranges[scan_angle[num]]

        self.get_logger().info(f"scan_data_: {self.scan_data_}")


    def update_cmd_vel(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular

        self.cmd_vel_pub_.publish(cmd_vel)
        
        self.get_logger().info(f"cmd_vel_pub_: {cmd_vel}")


    ######################
    ## Update functions ##
    ######################

    # Executed on a periodic basis.
    def update_callback(self):

        escape_range = math.radians(30)
        check_forward_dist = 0.7
        check_side_dist = 0.6

        # Behaviour depends on current state
        match self.turtlebot3_state_num:
            case State.GET_TB3_DIRECTION:
                self.get_logger().info("GET_TB3_DIRECTION")

                if self.scan_data_[CENTER] > check_forward_dist:
                    if self.scan_data_[LEFT] < check_side_dist:
                        self.prev_robot_pose_ = self.robot_pose_
                        self.turtlebot3_state_num = State.TB3_RIGHT_TURN
                    elif self.scan_data_[RIGHT] < check_side_dist:
                        self.prev_robot_pose_ = self.robot_pose_
                        self.turtlebot3_state_num = State.TB3_LEFT_TURN
                    else:
                        self.turtlebot3_state_num = State.TB3_DRIVE_FORWARD
                
                if self.scan_data_[CENTER] < check_forward_dist:
                    self.prev_robot_pose_ = self.robot_pose_
                    self.turtlebot3_state_num = State.TB3_RIGHT_TURN

            case State.TB3_DRIVE_FORWARD:
                self.get_logger().info("TB3_DRIVE_FORWARD")

                self.update_cmd_vel(LINEAR_VELOCITY, 0.0)
                self.turtlebot3_state_num = State.GET_TB3_DIRECTION

            case State.TB3_RIGHT_TURN:
                self.get_logger().info("TB3_RIGHT_TURN")

                if math.fabs(self.prev_robot_pose_ - self.robot_pose_) >= escape_range:
                    self.turtlebot3_state_num = State.GET_TB3_DIRECTION
                else:
                    self.update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY)

            case State.TB3_LEFT_TURN:
                self.get_logger().info("TB3_LEFT_TURN")

                if math.fabs(self.prev_robot_pose_ - self.robot_pose_) >= escape_range:
                    self.turtlebot3_state_num = State.GET_TB3_DIRECTION
                else:
                    self.update_cmd_vel(0.0, 1 * ANGULAR_VELOCITY)

            case _:
                self.turtlebot3_state_num = State.GET_TB3_DIRECTION


##########
## Main ##
##########

def main(args=None):
    rclpy.init(args=args)

    turtlebot3_drive_python = Turtlebot3DrivePython()

    rclpy.spin(turtlebot3_drive_python)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtlebot3_drive_python.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()