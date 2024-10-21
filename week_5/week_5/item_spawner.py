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

import os
import sys
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ament_index_python.packages import get_package_share_directory

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity, GetModelList, GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Twist

from auro_interfaces.srv import ItemRequest

class ItemSpawner(Node):

    def __init__(self):
        super().__init__('item_spawner')

        self.first_run = True
        self.spawn_new_item = True
        self.item_held = False

        models_path = os.path.join(get_package_share_directory("week_5"), "models")
        item_model_path = os.path.join(models_path, "item", "model.sdf")
        self.item_model = open(item_model_path, 'r').read()

        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity', callback_group=client_callback_group)
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list', callback_group=client_callback_group)
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state', callback_group=client_callback_group)
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state', callback_group=client_callback_group)

        self.pick_up_service = self.create_service(ItemRequest, '/pick_up_item', self.pick_up_item, callback_group=timer_callback_group)
        self.offload_service = self.create_service(ItemRequest, '/offload_item', self.offload_item, callback_group=timer_callback_group)

        self.timer = self.create_timer(0.1, self.control_loop, callback_group=timer_callback_group)


    def generate_item_pose(self):
        pose = Pose()
        pose.position.x = random.uniform(-2.0, 2.0)
        pose.position.y = random.uniform(-2.0, 2.0)
        self.get_logger().info(f"Generated item position: ({pose.position.x:.2f}, {pose.position.y:.2f})")        
        return pose


    def spawn_item(self):
        request = SpawnEntity.Request()
        request.name = "item"
        request.xml = self.item_model
        request.initial_pose = self.generate_item_pose()
        request.reference_frame = "world"

        self.get_logger().info("Spawning entity")
        return self.spawn_entity_client.call(request)


    def get_model_list(self):
        request = GetModelList.Request()
        return self.get_model_list_client.call(request)


    def get_entity_state(self, name):
        request = GetEntityState.Request()
        request.name = name
        return self.get_entity_state_client.call(request)
    

    def set_entity_state(self, name, pose):
        state = EntityState()
        state.name = name
        state.pose = pose
        state.twist = Twist()
        state.reference_frame = "world"

        request = SetEntityState.Request()
        request.state = state

        self.get_logger().info("Setting entity state")
        return self.set_entity_state_client.call(request)
    
    def pick_up_item(self, request, response):
        '''
        Makes the robot whose 'robot_id' passed in the request to pick an item in the vicinity.
        The 'success' field of response is set to True if pick up succeeded, and otherwise
        is set to False. The 'message' field provides further details.
        '''
        DISTANCE = 0.25

        response.success = False
        self.get_logger().info(f"Incoming request on pick_up_item from robot_id '{request.robot_id}'")

        if self.item_held:
            response.success = False
            response.message = f"Robot '{request.robot_id}' is already holding an item"
        else:
            model_list_msg = self.get_model_list()
            entity_state_msg = self.get_entity_state("waffle_pi")
            robot_position = entity_state_msg.state.pose.position

            if "item" in model_list_msg.model_names:

                entity_state_msg = self.get_entity_state("item")
                item_position = entity_state_msg.state.pose.position

                distance = math.sqrt((robot_position.x - item_position.x) ** 2 +
                                    (robot_position.y - item_position.y) ** 2)

                if distance < DISTANCE: # Approximate radius of item for collection.
                    self.get_logger().info("Collected item")
                    pose = Pose()
                    pose.position.x = robot_position.x
                    pose.position.y = robot_position.y
                    pose.position.z = 0.15
                    
                    self.set_entity_state("item", pose)
                    self.item_held = True

                    response.success = True
                    response.message = f"Robot '{request.robot_id}' collected item successfully"
                    return response
            
            response.success = False
            response.message = f"Robot '{request.robot_id}' unable to pick any item."
                    
        return response
    
    def offload_item(self, request, response):
        '''
        Offloads an item held by the robot_id passed in the request message.
        The 'success' field of response is set to True if offloading has succeeded, and otherwise
        is is set to False. The 'message' field provides further details.
        '''
        response.success = False
        self.get_logger().info('Incoming request on offload_item from robot_id: %s' % request.robot_id)

        if not self.item_held:
            response.success = False
            response.message = f"Robot '{request.robot_id}' does not hold any item, so unable to offload."
        else:
            model_list_msg = self.get_model_list()
            entity_state_msg = self.get_entity_state("waffle_pi")
            robot_position = entity_state_msg.state.pose.position
            
            if "item" in model_list_msg.model_names:

                # Need to put it back on the floor
                pose = Pose()
                pose.position.x = robot_position.x
                pose.position.y = robot_position.y
                pose.position.z = 0.0
                
                # World reference frame in set_entity_state
                self.set_entity_state("item", pose)

                self.item_held = False

                response.success = True
                response.message = f"Item held by robot '{request.robot_id}' has been offloaded in the arena."
        
        return response

    def control_loop(self):

        if self.first_run:
            self.first_run = False
            self.spawn_item()            

        model_list_msg = self.get_model_list()
        entity_state_msg = self.get_entity_state("waffle_pi")
        robot_position = entity_state_msg.state.pose.position

        if "item" in model_list_msg.model_names:

            if self.item_held: # Continuously update item's location in the world as the robot moves.
                entity_state_msg = self.get_entity_state("item")
                pose = Pose()
                pose.position.x = robot_position.x
                pose.position.y = robot_position.y
                pose.position.z = 0.15
                self.set_entity_state("item", pose)

def main(args=None):

    rclpy.init(args = args)

    node = ItemSpawner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt: # TODO: Why does this take 2x Ctrl+c to kill?
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()