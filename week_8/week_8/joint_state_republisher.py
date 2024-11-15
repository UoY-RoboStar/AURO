import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import JointState


class JointStateRepublisher(Node):

    def __init__(self):
        super().__init__('joint_state_republisher')

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)


    def joint_state_callback(self, msg):

        new_msg = msg

        republish = False

        for index, joint_name in enumerate(new_msg.name):
            if joint_name in ["wheel_left_joint", "wheel_right_joint"]:
                new_msg.name[index] = "true_" + joint_name
                republish = True
        
        if republish == True:
            self.joint_state_publisher.publish(new_msg)


def main(args=None):

    rclpy.init(args = args)

    node = JointStateRepublisher()

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