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

from geometry_msgs.msg import Twist, Vector3

import random

class TurtleController(Node):

    def __init__(self):

        # We give a name to this node
        super().__init__('turtle_controller')

        # Declare a publisher of type Twist, with topic name '/turtle1/cmd_vel' and queue_size of 10.
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Setup a periodic callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angular_z = 0.0
        self.direction = 1.0

    # This callback is called on a timer as defined above.
    def timer_callback(self):

        # Create a twist with two vectors, consisting of linear+angular parameters.
        msg = Twist(linear = Vector3(x = 1.0,
                                     y = 0.0,
                                     z = 0.0),
                    angular = Vector3(x = 0.0,
                                      y = 0.0,
                                      z = self.angular_z))

        # Publish the twist
        self.publisher.publish(msg)

        # Output logging info
        self.get_logger().info(f"Publishing: Twist - linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")

        # Change direction randomly
        if abs(self.angular_z) > random.uniform(1.0, 2.0):
            self.angular_z = 0.0
            self.direction = 1.0 if random.random() < 0.5 else -1.0            
        else:
            self.angular_z += 0.5 * self.direction


def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()