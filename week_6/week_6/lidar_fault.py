import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import LaserScan

import random

class LiDARFault(Node):

    def __init__(self):
        super().__init__('lidar_fault')
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)

        self.scan_publisher = self.create_publisher(LaserScan, '/robot1/scan_faulty', 10)

    def scan_callback(self, msg):
        msg_faulty = msg

        fault_probability = 0.5

        if random.random() < fault_probability:
            for i, range in enumerate(msg_faulty.ranges):
                msg_faulty.ranges[i] = float('inf') # Never detect an obstacle
                # msg_faulty.ranges[i] = 0 # Always detect an obstacle
                # msg_faulty.ranges[i] = random.gauss(msg_faulty.ranges[i], 0.2) # Gaussian noise centred on true reading, with some standard deviation
                # msg_faulty.ranges[i] = msg_faulty.ranges[i] / 2 # Obstacles appear twice as close as they really are

        self.scan_publisher.publish(msg_faulty)


def main(args=None):

    rclpy.init(args = args)

    node = LiDARFault()

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