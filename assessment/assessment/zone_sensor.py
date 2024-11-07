#
# Copyright (c) 2024 University of York and others
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation
#
# ZoneSensor that can detect zones according to preset colour using the camera.
#

import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image
from assessment_interfaces.msg import Zone, ZoneList

# Allows to convert between ROS2 image messages and OpenCV images
import cv2
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum
import numpy as np

class Colour(Enum):
    CYAN = 1
    PURPLE = 2
    GREEN = 3
    PINK = 4

class ZoneSensor(Node):

    def __init__(self):
        super().__init__('zone_sensor')       

        # Controls whether to output debug image mask for visualisation, eg. RViz
        self.declare_parameter('image_mask', False)
        self.image_mask = self.get_parameter('image_mask').value

        self.bridge = CvBridge()

        # Setup colours
        self.cyan = cv2.cvtColor(np.uint8([[[0,255,255]]]),cv2.COLOR_RGB2HSV)
        self.purple = cv2.cvtColor(np.uint8([[[66,0,255]]]),cv2.COLOR_RGB2HSV)
        self.green = cv2.cvtColor(np.uint8([[[46,139,87]]]),cv2.COLOR_RGB2HSV)
        self.pink = cv2.cvtColor(np.uint8([[[255,20,147]]]),cv2.COLOR_RGB2HSV)

        # Creates subscriber on the raw image camera coming from the Turtlebot.
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        # The following published topic indicates the zones visible.
        self.zone_list_publisher = self.create_publisher(ZoneList, 'zone', 10)

        # The following publishes the camera image annotated with zone information.
        self.image_publisher = self.create_publisher(Image, 'camera/image_zone', 10)

        # The following publishes the image mask as seen for zone indentification.
        self.image_mask_publisher = self.create_publisher(Image, 'camera/image_mask', 10)

    def image_callback(self, data):

        zone_list = ZoneList()

        # Converts from OpenCV to ROS format.
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")

        image_width = frame.shape[1]
        image_height = frame.shape[0]

        augmented = frame.copy()

        # Converts frame colourspace from BGR to HSV (Hue, Saturation, Value).
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        

        SATURATION_LOWER = 100
        SATURATION_UPPER = 255
        VALUE_LOWER = 20
        VALUE_UPPER = 255

        lower = np.array([80, SATURATION_LOWER, VALUE_LOWER])
        upper = np.array([100, SATURATION_UPPER, VALUE_UPPER])

        # For image segmentation, sets up masks in HSV colour space.
        cyan_mask = cv2.inRange(hsv, np.array([self.cyan[0][0][0], 100, 20]), np.array([self.cyan[0][0][0], 255,255]))
        purple_mask = cv2.inRange(hsv, np.array([self.purple[0][0][0], 100, 20]), np.array([self.purple[0][0][0]+1, 255,255]))
        green_mask = cv2.inRange(hsv, np.array([self.green[0][0][0], 100, 20]), np.array([self.green[0][0][0], 255,255]))
        pink_mask = cv2.inRange(hsv, np.array([self.pink[0][0][0]-3, 100, 20]), np.array([self.pink[0][0][0], 255,255]))        

        for colour in Colour:

            match colour:
                case colour.CYAN:
                    mask = cyan_mask
                    zone = Zone.ZONE_CYAN
                    text = "CYAN ZONE"
                case colour.PURPLE:
                    mask = purple_mask
                    zone = Zone.ZONE_PURPLE
                    text = "PURPLE ZONE"
                case colour.GREEN:
                    mask = green_mask
                    zone = Zone.ZONE_GREEN
                    text = "GREEN ZONE"
                case colour.PINK:
                    mask = pink_mask
                    zone = Zone.ZONE_PINK
                    text = "PINK ZONE"
                case _:
                    pass

            msg = Zone()
            msg.zone = zone

            moments = cv2.moments(mask)

            if moments["m00"] != 0:

                centre_x = int(moments["m10"] / moments["m00"])
                centre_y = int(moments["m01"] / moments["m00"])

                black = (0, 0, 0)
                white = (255, 255, 255)

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1
                font_thickness = 2
                text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
                text_position = (int(centre_x - text_size[0] / 2), int(centre_y + text_size[1] * 2))

                cv2.putText(augmented, text, text_position, font, font_scale, black, font_thickness * 4, cv2.LINE_AA)
                cv2.putText(augmented, text, text_position, font, font_scale, white, font_thickness, cv2.LINE_AA)

                cv2.circle(augmented, (centre_x, centre_y), 10, black, -1, lineType=cv2.LINE_AA)
                cv2.circle(augmented, (centre_x, centre_y), 6, white, -1, lineType=cv2.LINE_AA)

                msg.x = int((image_width / 2) - centre_x)
                msg.y = int((image_height / 2) - centre_y)

                # Ratio of nonzero pixels in mask vs all pixels.
                # When 1.0 means all pixels in mask match, ie. at home.
                msg.size = np.count_nonzero(mask) / (image_width * image_height)
                zone_list.data.append(msg)

        self.zone_list_publisher.publish(zone_list)

        try:
            image_msg = self.bridge.cv2_to_imgmsg(augmented, encoding="bgr8")
            image_msg.header = data.header
            self.image_publisher.publish(image_msg)
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")

        if self.image_mask:
            try:
                image_msg = self.bridge.cv2_to_imgmsg(cyan_mask + purple_mask + green_mask + pink_mask, encoding="mono8")
                image_msg.header = data.header
                self.image_mask_publisher.publish(image_msg)
            except CvBridgeError as e:
                self.get_logger().info(f"CvBridgeError: {e}")


def main(args=None):

    rclpy.init(args = args)

    node = ZoneSensor()

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
