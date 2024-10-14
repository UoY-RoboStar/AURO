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

import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from auro_interfaces.msg import StringWithPose

from tf_transformations import euler_from_quaternion
import angles

from enum import Enum
import random
import math

LINEAR_VELOCITY  = 0.3
ANGULAR_VELOCITY = 0.5

class State(Enum):
    STARTING = 0
    FORWARD = 1
    TURNING_LEFT = 2
    TURNING_RIGHT = 3

class TurtleBot3RandomWalk(Node):

    def __init__(self):
        super().__init__('turtlebot3_random_walk')

        self.state = State.STARTING
        self.pose = Pose()
        self.previous_pose = Pose()
        self.yaw = 0.0
        self.previous_yaw = 0.0

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.odom_subscriber  # prevent unused variable warning

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_publisher = self.create_publisher(StringWithPose, '/marker_input', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.time_elapsed = self.get_clock().now()

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw


    def control_loop(self):

        # Outputs current state and pose.
        marker_input = StringWithPose()
        marker_input.text = str(self.state)
        marker_input.pose = self.pose
        self.marker_publisher.publish(marker_input)

        match self.state:

            case State.STARTING:
                delay = 3
                time_difference = self.get_clock().now() - self.time_elapsed
                self.get_logger().info(f"Starting robot controller in {delay} seconds...")

                if time_difference > Duration(seconds = delay):
                    self.get_logger().info(f"STARTED")
                    self.previous_pose = self.pose
                    self.state = State.FORWARD

            case State.FORWARD:
                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.publisher.publish(msg)
                self.get_logger().info(f"Forward: {msg}")

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance = math.sqrt(difference_x ** 2 + difference_y ** 2)

                if distance > 1:
                    self.time_elapsed = self.get_clock().now()
                    self.previous_yaw = self.yaw
                    self.state = random.choice([State.TURNING_LEFT, State.TURNING_RIGHT])

            case State.TURNING_LEFT:
                msg = Twist()
                msg.angular.z = ANGULAR_VELOCITY
                self.publisher.publish(msg)
                self.get_logger().info(f"Turning left: {msg}")

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)

                if math.fabs(yaw_difference) >= math.radians(90):
                    self.time_elapsed = self.get_clock().now()
                    self.previous_pose = self.pose
                    self.state = State.FORWARD

            case State.TURNING_RIGHT:
                msg = Twist()
                msg.angular.z = -ANGULAR_VELOCITY
                self.publisher.publish(msg)
                self.get_logger().info(f"Turning right: {msg}")

                if (self.get_clock().now() - self.time_elapsed) > Duration(seconds = random.uniform(0.5, 3.0)):
                    self.time_elapsed = self.get_clock().now()
                    self.previous_pose = self.pose
                    self.state = State.FORWARD

            case _:
                pass
        

    def destroy_node(self):
        msg = Twist()
        self.publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = TurtleBot3RandomWalk()

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