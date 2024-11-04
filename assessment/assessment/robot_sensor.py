#
# Copyright (c) 2024 University of York and others
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation
#
# Node implementing sensor to detect other robots.
#

import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image
from assessment_interfaces.msg import Robot, RobotList

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class RobotSensor(Node):

    def __init__(self):
        super().__init__('robot_sensor')

        self.bridge = CvBridge()

        # Colour of robots for image segmentation
        self.robot_colour = cv2.cvtColor(np.uint8([[[255,0,255]]]),cv2.COLOR_RGB2HSV)

        # Subscribes to camera image
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        # Publishes the list of robots visible
        self.robot_publisher = self.create_publisher(RobotList, 'robots', 10)

        # Publishes camera image with text marking each detected robot
        self.image_publisher = self.create_publisher(Image, 'camera/image_robots', 10)

    def image_callback(self, data):

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")

        image_width = frame.shape[1]
        image_height = frame.shape[0]

        augmented = frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        

        SATURATION_LOWER = 100
        SATURATION_UPPER = 255
        VALUE_LOWER = 20
        VALUE_UPPER = 255

        lower = np.array([self.robot_colour[0][0][0], SATURATION_LOWER, VALUE_LOWER])
        upper = np.array([self.robot_colour[0][0][0], SATURATION_UPPER, VALUE_UPPER])

        mask = cv2.inRange(hsv, lower, upper)

        robot_list = RobotList()

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:  
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            area = cv2.contourArea(contour)

            if len(approx) >= 4:

                moments = cv2.moments(contour)

                if moments["m00"] != 0:
                    centre_x = int(moments["m10"] / moments["m00"])
                    centre_y = int(moments["m01"] / moments["m00"])

                    black = (0, 0, 0)
                    white = (255, 255, 255)

                    text = "ROBOT"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 1
                    font_thickness = 2
                    text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
                    text_position = (int(centre_x - text_size[0] / 2), int(centre_y + text_size[1] * 2))

                    cv2.putText(augmented, text, text_position, font, font_scale, black, font_thickness * 4, cv2.LINE_AA)
                    cv2.putText(augmented, text, text_position, font, font_scale, white, font_thickness, cv2.LINE_AA)

                    cv2.circle(augmented, (centre_x, centre_y), 10, black, -1, lineType=cv2.LINE_AA)
                    cv2.circle(augmented, (centre_x, centre_y), 6, white, -1, lineType=cv2.LINE_AA)

                    msg = Robot()
                    msg.x = int((image_width / 2) - centre_x)
                    msg.y = int((image_height / 2) - centre_y)
                    msg.size = np.count_nonzero(mask) / (image_width * image_height)
                    robot_list.data.append(msg)

        self.robot_publisher.publish(robot_list)

        try:
            image_msg = self.bridge.cv2_to_imgmsg(augmented, encoding="bgr8")
            image_msg.header = data.header
            self.image_publisher.publish(image_msg)
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")


def main(args=None):

    rclpy.init(args = args)

    node = RobotSensor()

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