#
# Copyright (c) 2024 University of York and others
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation
#
# Item sensor node. Provides the ability to detect items of different colours using the camera.
# 
 
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image
from assessment_interfaces.msg import Item, ItemList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import cv2
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np
from enum import Enum

class Colour(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3


class ItemSensor(Node):

    def __init__(self):
        super().__init__('item_sensor')

        self.item_values = {}
        self.item_values[Colour.RED] = 5
        self.item_values[Colour.GREEN] = 10
        self.item_values[Colour.BLUE] = 15

        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.item_publisher = self.create_publisher(ItemList, 'items', 10)
        self.image_publisher = self.create_publisher(Image, 'camera/image_items', 10)


    def image_callback(self, data):

        item_list = ItemList()

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")

        image_width = frame.shape[1]
        image_height = frame.shape[0]

        contours_frame = frame.copy()
        overlay = frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        SATURATION_LOWER = 100
        SATURATION_UPPER = 255
        VALUE_LOWER = 20
        VALUE_UPPER = 255

        red_lower1 = np.array([0, SATURATION_LOWER, VALUE_LOWER])
        red_upper1 = np.array([10, SATURATION_UPPER, VALUE_UPPER])

        red_lower2 = np.array([170, SATURATION_LOWER, VALUE_LOWER])
        red_upper2 = np.array([180, SATURATION_UPPER, VALUE_UPPER])

        red_lower_mask = cv2.inRange(hsv, red_lower1, red_upper1)
        red_upper_mask = cv2.inRange(hsv, red_lower2, red_upper2)

        red_mask = red_lower_mask + red_upper_mask

        green_lower = np.array([45, SATURATION_LOWER, VALUE_LOWER])
        green_upper = np.array([85, SATURATION_UPPER, VALUE_UPPER])

        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        blue_lower = np.array([110, SATURATION_LOWER, VALUE_LOWER])
        blue_upper = np.array([130, SATURATION_UPPER, VALUE_UPPER])

        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

        for colour in Colour:
            match colour:
                case colour.RED:
                    mask = red_mask
                case colour.GREEN:
                    mask = green_mask
                case colour.BLUE:
                    mask = blue_mask
                case _:
                    pass

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:  
                epsilon = 0.01 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                area = cv2.contourArea(contour)

                if len(approx) > 8 and cv2.isContourConvex(approx):

                    intersects_border = False
                    
                    for point in contour:
                        x, y = point[0]
                        if x == 0 or x == image_width - 1 or y == 0 or y == image_height - 1:
                            intersects_border = True
                            break

                    if intersects_border: # Ignore contours that are partially occluded by the edge of the camera frame
                        continue

                    moments = cv2.moments(contour)

                    if moments["m00"] != 0:
                        centre_x = int(moments["m10"] / moments["m00"])
                        centre_y = int(moments["m01"] / moments["m00"])

                        black = (0, 0, 0)
                        white = (255, 255, 255)

                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        centre = (int(x), int(y))
                        radius = int(radius)
                        cv2.circle(overlay, centre, radius, black, -1, lineType=cv2.LINE_AA)

                        msg = Item()
                        msg.x = int((image_width / 2) - x)
                        msg.y = int((image_height / 2) - y)
                        msg.diameter = radius * 2
                        msg.colour = colour.name
                        msg.value = self.item_values[colour]
                        item_list.data.append(msg)

                        text = f"{self.item_values[colour]}"
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 1
                        font_thickness = 2
                        text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
                        text_position = (int(centre_x - text_size[0] / 2), int(centre_y + text_size[1] / 2))

                        cv2.putText(contours_frame, text, text_position, font, font_scale, black, font_thickness * 4, cv2.LINE_AA)
                        cv2.putText(contours_frame, text, text_position, font, font_scale, white, font_thickness, cv2.LINE_AA)

        alpha = 0.3
        contours_frame = cv2.addWeighted(overlay, alpha, contours_frame, 1 - alpha, 0)

        self.item_publisher.publish(item_list)

        try:
            image_msg = self.bridge.cv2_to_imgmsg(contours_frame, encoding="bgr8")
            image_msg.header = data.header
            self.image_publisher.publish(image_msg)
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")


def main(args=None):

    rclpy.init(args = args)

    node = ItemSensor()

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