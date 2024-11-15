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

import sys
import argparse
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

BUFFER_SIZE = 1000

class PathPublisher(Node):

    def __init__(self, args):
        super().__init__('path_publisher')

        parser = argparse.ArgumentParser()
        parser.add_argument('-x', type=float, metavar='INITIAL_X', help='Initial x position', required=True)
        parser.add_argument('-y', type=float, metavar='INITIAL_Y', help='Initial y position', required=True)
        self.args = parser.parse_args(args[1:])

        self.odom_pose_history = deque(maxlen = BUFFER_SIZE)
        self.true_pose_history = deque(maxlen = BUFFER_SIZE)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.true_pose_subscriber = self.create_subscription(
            Odometry,
            '/world',
            self.true_pose_callback,
            10)
        
        self.odom_path_publisher = self.create_publisher(Path, '/odom_path', 10)
        self.true_path_publisher = self.create_publisher(Path, '/true_path', 10)

    
    def odom_callback(self, msg):
        pose = msg.pose.pose

        pose.position.x += self.args.x
        pose.position.y += self.args.y

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        self.odom_pose_history.append(pose_stamped)

        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.odom_pose_history

        self.odom_path_publisher.publish(path)


    def true_pose_callback(self, msg):
        pose = msg.pose.pose

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        self.true_pose_history.append(pose_stamped)

        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.true_pose_history

        self.true_path_publisher.publish(path)


def main(args=sys.argv):

    rclpy.init(args = args)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    node = PathPublisher(args_without_ros)

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