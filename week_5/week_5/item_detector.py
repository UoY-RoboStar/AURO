#
# Copyright (c) 2024 University of York and others
#
# This program and the accompanying materials are made available under the
# terms of the BSD License 2.0
# 
# SPDX-License-Identifier: BSD-2.0
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation


import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image
from auro_interfaces.msg import Item, ItemList

import cv2
from cv_bridge import CvBridge

import numpy as np

class ItemDetector(Node):

    def __init__(self):
        super().__init__('item_detector')

        self.br = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.item_publisher = self.create_publisher(ItemList, 'items', 10)


    def image_callback(self, data):

        item_list = ItemList()

        self.get_logger().info('Received camera image')

        frame = self.br.imgmsg_to_cv2(data, "bgr8")
        # cv2.imshow("/camera/image_raw", frame)

        result = frame.copy()
        contours_frame = frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # cv2.imshow("HSV", hsv)

        # https://cvexplained.wordpress.com/2020/04/28/color-detection-hsv/

        # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([10, 255, 255])

        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([160, 100, 20])
        upper2 = np.array([179, 255, 255])

        lower_mask = cv2.inRange(hsv, lower1, upper1)
        upper_mask = cv2.inRange(hsv, lower2, upper2)

        full_mask = lower_mask + upper_mask

        result = cv2.bitwise_and(result, result, mask=full_mask)

        # cv2.imshow('mask', full_mask)
        # cv2.imshow('result', result)

        contours, hierarchy = cv2.findContours(full_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:  
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            area = cv2.contourArea(contour)

            if len(approx) > 8 and cv2.isContourConvex(approx):

                intersects_border = False
                
                for point in contour:
                    x, y = point[0]
                    if x == 0 or x == 640 - 1 or y == 0 or y == 480 - 1:
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
                    cv2.circle(contours_frame, centre, radius, black, 2 * 4, lineType=cv2.LINE_AA)
                    cv2.circle(contours_frame, centre, radius, white, 2, lineType=cv2.LINE_AA)

                    msg = Item()
                    msg.x = int((contours_frame.shape[1] / 2) - x)
                    msg.diameter = radius * 2
                    msg.colour = "red"
                    item_list.data.append(msg)

                    text = f"x: {msg.x}, diameter: {msg.diameter}"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 1
                    font_thickness = 2
                    text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
                    text_position = (int(centre_x - text_size[0] / 2), int(centre_y + text_size[1] * 2))

                    cv2.putText(contours_frame, text, text_position, font, font_scale, black, font_thickness * 4, cv2.LINE_AA)
                    cv2.putText(contours_frame, text, text_position, font, font_scale, white, font_thickness, cv2.LINE_AA)

                    cv2.circle(contours_frame, (centre_x, centre_y), 10, black, -1, lineType=cv2.LINE_AA)
                    cv2.circle(contours_frame, (centre_x, centre_y), 6, white, -1, lineType=cv2.LINE_AA)

        self.item_publisher.publish(item_list)
        self.get_logger().info(f"Publishing: {item_list}")

        cv2.imshow("Contours", contours_frame)

        cv2.waitKey(1)

def main(args=None):

    rclpy.init(args = args)

    node = ItemDetector()

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