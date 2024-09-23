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

from geometry_msgs.msg import Twist


# This listens to the twists published by the other node.
class TurtleListener(Node):

    def __init__(self):
        super().__init__('turtle_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    # The listener callback prints out information as received.
    def listener_callback(self, msg):
        self.get_logger().info(f"Received: Twist - linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")


def main(args=None):
    rclpy.init(args=args)

    turtle_listener = TurtleListener()

    rclpy.spin(turtle_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()