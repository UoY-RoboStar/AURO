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
#   * Pedro Ribeiro - revised implementation
#
# Implementation of RoboChart controller 'ObstacleDetector' with single Finite State machine
# 'DetectorFSM' as in the project in 'robochart/turtlebot-robochart-move-composite'.
 
import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from auro_interfaces.msg import StringWithPose, Item, ItemList, Direction

from tf_transformations import euler_from_quaternion
import angles

from enum import Enum
import random
import math

SCAN_THRESHOLD = 0.4 # Metres per second
 # Array indexes for sensor sectors
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

# Finite state machine (FSM) states
class State(Enum):
    INITIAL = 0
    IDLE = 1

class DirectionTrigger():
    def __init__(self, left, right, front, back):
        self.left = left
        self.right = right
        self.front = front
        self.back = back

    def __repr__(self):
        return f'({self.left}, {self.right}, {self.front}, {self.back})'
    
class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Class variables used to store persistent values between executions of callbacks and control loop
        self.state = State.INITIAL # Current FSM state
        
        self.scan_triggered = False
        self.data = []
        self.dir = {}

        # Subscribes to LaserScan messages on the /scan topic
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
        #
        # QoSPresetProfiles.SENSOR_DATA specifices "best effort" reliability and a small queue size
        # https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos.py#L455
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos.py#L428-L431
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)

        self.obstacle_publisher = self.create_publisher(Direction, '/obstacle', 10)

        # Creates a timer that calls the control_loop method repeatedly - each loop represents single iteration of the FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    # Called every time scan_subscriber recieves a LaserScan message from the /scan topic
    #
    # The Gazebo RaySensor calculates distance at which rays intersect with obstacles
    # The data is published by the Gazebo ROS ray sensor plugin
    # https://github.com/gazebosim/gazebo-classic/tree/gazebo11/gazebo/sensors
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_ray_sensor.cpp#L178-L205
    #
    # This plugin is configured to match the LiDAR on the TurtleBot3 in the SDF file that defines the robot model
    # http://wiki.ros.org/hls_lfcd_lds_driver
    # https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble-devel/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf#L132-L165
    def scan_callback(self, msg):
        self.scan_triggered = True
        self.data = msg.ranges

    # Control loop for the FSM - called periodically by self.timer
    def control_loop(self):
        
        match self.state:

            case State.INITIAL:
                self.state = State.IDLE
                return

            case State.IDLE:
                if self.scan_triggered:
                    self.dir = DirectionTrigger(
                                front=min(self.data[331:359] + self.data[0:30]) < SCAN_THRESHOLD, # front, 30 to 331 degrees (30 to -30 degrees)
                                left=min(self.data[31:90])  < SCAN_THRESHOLD,    # left, 31 to 90 degrees (31 to 90 degrees)
                                back=min(self.data[91:270])  < SCAN_THRESHOLD ,  #back, 91 to 270 degrees (91 to -90 degrees)
                                right=min(self.data[271:330]) < SCAN_THRESHOLD   # right, 271 to 330 degrees (-30 to -91 degrees)
                    )
                    
                    # Transitions out of IDLE, with simplified implementation of junction.
                    if self.dir.front:
                        self.obstacle_publisher.publish(Direction(direction=Direction.FRONT))
                    elif self.dir.left:
                        self.obstacle_publisher.publish(Direction(direction=Direction.LEFT))
                    elif self.dir.right:
                        self.obstacle_publisher.publish(Direction(direction=Direction.RIGHT))

            case _:
                pass
        

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()