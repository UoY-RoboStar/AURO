#
# Copyright (c) 2024 University of York and others
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation
#
# ItemManager node for simulating items in the arena, their collection and offloading.
#

import os
import sys
import math
import random
import argparse
import multiprocessing

from enum import Enum
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration

from ament_index_python.packages import get_package_share_directory

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity, GetModelList, GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Twist
from assessment_interfaces.msg import ItemHolder, ItemHolders, ItemLog
from auro_interfaces.srv import ItemRequest

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

HOME_ZONE_MIN_X = -4.0
HOME_ZONE_MAX_X = -3.0
HOME_ZONE_MIN_Y = -3.0
HOME_ZONE_MAX_Y =  3.0

class Colour(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3

class ZoneLocation(Enum):
    TOP_LEFT = 1
    TOP_RIGHT = 2
    BOTTOM_RIGHT = 3
    BOTTOM_LEFT = 4

class Zone():
    def __init__(self, location, x, y):
        self.location = location
        self.x = x
        self.y = y
        self.items_returned = {}

    def __repr__(self):
        return f'(location: {self.location}, x: {self.x}, y: {self.y}, items_returned: {self.items_returned})'

class Item():
    def __init__(self, x, y, colour, cluster_id):
        self.x = x
        self.y = y
        self.colour = colour
        self.cluster_id = cluster_id

    def __repr__(self):
        return f'({self.x}, {self.y})'


class Cluster():
    def __init__(self, x, y, colour):
        self.x = x
        self.y = y
        self.colour = colour

    def __repr__(self):
        return f'({self.x}, {self.y})'


class Robot():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.item_held = None
        self.previous_item_held = None

    def __repr__(self):
        return f'({self.x}, {self.y}, {self.item_held})'


class ItemManager(Node):

    def __init__(self, args):
        super().__init__('item_manager')

        parser = argparse.ArgumentParser()
        group = parser.add_mutually_exclusive_group(required=True)
        group.add_argument('--random_seed', type=int, metavar='RANDOM_SEED', help='Random seed')
        self.args = parser.parse_args(args[1:])

        self.declare_parameter('zone_top_left', True)
        self.declare_parameter('zone_top_right', True)
        self.declare_parameter('zone_bottom_left', True)
        self.declare_parameter('zone_bottom_right', True)

        zone_top_left = self.get_parameter('zone_top_left').value
        zone_top_right = self.get_parameter('zone_top_right').value
        zone_bottom_left = self.get_parameter('zone_bottom_left').value
        zone_bottom_right = self.get_parameter('zone_bottom_right').value

        random.seed(self.args.random_seed)

        self.first_run = True
        self.previous_time = self.get_clock().now()

        self.clusters = {}
        self.robots = {}
        self.items = {}

        # Initialize zones
        self.zones = {}
        if zone_top_left:
            self.zones[ZoneLocation.TOP_LEFT] = Zone(ZoneLocation.TOP_LEFT, 2.57, 2.5)
        if zone_top_right:
            self.zones[ZoneLocation.TOP_RIGHT] = Zone(ZoneLocation.TOP_RIGHT, 2.57, -2.46)
        if zone_bottom_right:
            self.zones[ZoneLocation.BOTTOM_RIGHT] = Zone(ZoneLocation.BOTTOM_RIGHT, -3.42, -2.46)
        if zone_bottom_left:
            self.zones[ZoneLocation.BOTTOM_LEFT] = Zone(ZoneLocation.BOTTOM_LEFT, -3.42, 2.5)
        # self.zones[ZoneLocation.TOP_LEFT] = Zone(ZoneLocation.TOP_LEFT, 5, 0.0128)
        # self.zones[ZoneLocation.TOP_RIGHT] = Zone(ZoneLocation.TOP_RIGHT, 5, -4.967)
        # self.zones[ZoneLocation.BOTTOM_RIGHT] = Zone(ZoneLocation.BOTTOM_RIGHT, -1, 4.967)
        # self.zones[ZoneLocation.BOTTOM_LEFT] = Zone(ZoneLocation.BOTTOM_LEFT, -1, 0.0128)

        self.items_returned = {}

        self.cluster_counter = 0
        self.item_counter = 0

        self.item_values = {}
        self.item_values[Colour.RED] = 5
        self.item_values[Colour.GREEN] = 10
        self.item_values[Colour.BLUE] = 15

        self.item_models = {}

        for colour in Colour:
            for _, zone in self.zones.items():
                zone.items_returned[colour] = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        models_path = os.path.join(get_package_share_directory("assessment"), "models")
        item_model_path = os.path.join(models_path, "item", "model.sdf")

        tree = ET.parse(item_model_path)
        root = tree.getroot()

        self.item_models[Colour.RED] = ET.tostring(root, encoding='unicode')

        for node in root.iter("name"):
            for element in node.iter():
                element.text = "green_outlined"

        self.item_models[Colour.GREEN] = ET.tostring(root, encoding='unicode')

        for node in root.iter("name"):
            for element in node.iter():
                element.text = "blue_outlined"

        self.item_models[Colour.BLUE] = ET.tostring(root, encoding='unicode')

        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        publisher_callback_group = MutuallyExclusiveCallbackGroup()

        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity', callback_group=client_callback_group)
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list', callback_group=client_callback_group)
        self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state', callback_group=client_callback_group)
        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state', callback_group=client_callback_group)

        self.timer = self.create_timer(0.1, self.control_loop, callback_group=timer_callback_group)

        self.item_log_publisher = self.create_publisher(ItemLog, '/item_log', 10, callback_group=publisher_callback_group)
        self.item_holders_publisher = self.create_publisher(ItemHolders, '/item_holders', 10, callback_group=publisher_callback_group)

        # Provide services to collect item or offload item, but ensure no concurrency with regular control loop
        self.pick_up_service = self.create_service(ItemRequest, '/pick_up_item', self.pick_up_item, callback_group=timer_callback_group)
        self.offload_service = self.create_service(ItemRequest, '/offload_item', self.offload_item, callback_group=timer_callback_group)
    
    def pick_up_item(self, request, response):
        '''
        Makes the robot whose 'robot_id' passed in the request to pick an item in the vicinity.
        The 'success' field of response is set to True if pick up succeeded, and otherwise
        is set to False. The 'message' field provides further details.
        '''
        DISTANCE = 0.35

        response.success = False
        self.get_logger().info(f"Incoming request on pick_up_item from robot_id '{request.robot_id}'")

        # Get robot_id and check that it exists.
        if request.robot_id not in self.robots:
            response.success = False
            response.message = f"Unable to find robot_id '{request.robot_id}'"
        else:
            robot = self.robots[request.robot_id]

            if robot.item_held is not None:
                response.success = False
                response.message = f"Robot '{request.robot_id}' is already holding an item"
            else:
                for item_id, item in self.items.items():
                    # self.get_logger().info(f"Item '{item_id}': {item}")

                    # Collects item if any is close enough.
                    if math.dist((robot.x, robot.y), (item.x, item.y)) < DISTANCE:
                        self.get_logger().info(f'{request.robot_id} collected {item_id}')
                        robot.item_held = item_id
                        response.success = True
                        response.message = f"Robot '{request.robot_id}' collected item successfully"
                        return response

                if robot.item_held is None:
                    response.success = False
                    response.message = f"Robot '{request.robot_id}' unable to pick any item."
                    
        return response

    async def offload_item(self, request, response):
        '''
        Offloads an item held by the robot_id passed in the request message.
        The 'success' field of response is set to True if offloading has succeeded, and otherwise
        is is set to False. The 'message' field provides further details.
        '''
        response.success = False
        self.get_logger().info('Incoming request on offload_item from robot_id: %s' % request.robot_id)

        # Get robot_id and check that it exists.
        if request.robot_id not in self.robots:
            response.success = False
            response.message = f"Unable to find robot_id '{request.robot_id}'"
        else:
            robot = self.robots[request.robot_id]

            if robot.item_held is None:
                response.success = False
                response.message = f"Robot '{request.robot_id}' does not hold any items, so unable to offload."
            else:
                zone = self.get_robot_zone(robot)
                self.get_logger().info(f"Robot is trying to offload '{robot.item_held}' at {zone}")

                if not self.allowed_drop_in_zone(robot, zone):                    
                    # Need to put it back on the floor
                    pose = Pose()
                    pose.position.x = 0.0
                    pose.position.y = 0.0
                    pose.position.z = 0.0
                    
                    # Use reference frame of the robot
                    await self.set_entity_state(robot.item_held, request.robot_id, pose)
                    
                    # We drop it on the floor if we are in a non-zone
                    response.success = True
                    response.message = f"Item '{robot.item_held}' held by robot '{request.robot_id}' has been offloaded in the arena."
                    robot.item_held = None
                    return response
                else:
                    # Update zone count of number of items of this colour collected.
                    zone.items_returned[self.items[robot.item_held].colour] += 1
                    x, y = self.generate_item_position(self.items[robot.item_held].cluster_id)

                    # Update stored item position
                    self.items[robot.item_held].x = x
                    self.items[robot.item_held].y = y

                    # Update item location in simulation at absolute position, ie. world frame.
                    pose = Pose()
                    pose.position.x = self.items[robot.item_held].x
                    pose.position.y = self.items[robot.item_held].y
                    pose.position.z = 0.0
                    
                    await self.set_entity_state(robot.item_held, 'world', pose)

                    robot.item_held = None
                    robot.previous_item_held = None

                    response.message = f"Item '{robot.item_held}' held by robot '{request.robot_id}' has been offloaded at zone '{zone.location}'."
                    response.success = True
        
        return response
    
    def get_robot_zone(self, robot):
        '''Returns the zone the robot is currently in, and None if it is not anywhere near a zone.'''
        
        # Half-length of square, centered at (x,y) of zone.
        ZONE_SIZE = 0.5

        # Assuming disjoint zones
        for _, zone in self.zones.items():
            if robot.x <= (zone.x + ZONE_SIZE) and robot.x >= (zone.x - ZONE_SIZE) and robot.y <= (zone.y + ZONE_SIZE) and robot.y >= (zone.y - ZONE_SIZE):
                return zone

        return None

    def allowed_drop_in_zone(self, robot, zone):
        '''Returns whether the robot is allowed to drop an item in the specified zone.'''
        if zone is None:
            return False
        else:
            match_colour = None

            for colour, count in zone.items_returned.items():
                if count >= 1:
                    match_colour = colour
                    break

            if match_colour == None or match_colour == self.items[robot.item_held].colour:
                return True
            else:
                return False

    def generate_cluster_location(self, colour):
        while True:
            # Randomly generate a candidate cluster location
            y = random.randint(-2, 2)
            match colour:
                case Colour.RED:
                    x = random.randint(-2, -1)
                case Colour.GREEN:
                    x = random.randint(-1, 1)
                case Colour.BLUE:
                    x = random.randint(1, 2)
                case _:
                    pass

            # Invalid if it clashes with an obstacle
            if (x == 1 and y == 1) or (x == 1 and y == -1) or (x == -1 and y == 1) or (x == -1 and y == -1):
                continue

            # Invalid if it clashes with an existing cluster
            for cluster in self.clusters.values():
                if (cluster.x == x and cluster.y == y) or math.dist((cluster.x, cluster.y), (x, y)) <= 1.0:
                    break
            else:                    
                return x, y

    
    def generate_item_position(self, cluster_id):
        while True:

            radius = random.uniform(0, 0.5)
            angle = math.radians(random.uniform(0, 360))

            x = self.clusters[cluster_id].x + round(radius * math.cos(angle), 2)
            y = self.clusters[cluster_id].y + round(radius * math.sin(angle), 2)

            for item in self.items.values():
                if item.cluster_id == cluster_id and math.dist((item.x, item.y), (x, y)) < 0.3:
                    break
            else:
                return x, y

    def spawn_item(self, name, x, y, colour, z = 0.0):

        while not self.spawn_entity_client.wait_for_service():
            pass

        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.item_models[colour]
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.reference_frame = "world"
        self.item_counter += 1
        return self.spawn_entity_client.call_async(request)


    def get_model_list(self):

        while not self.get_model_list_client.wait_for_service():
            pass

        request = GetModelList.Request()
        return self.get_model_list_client.call_async(request)

    def get_entity_state(self, name):

        while not self.get_entity_state_client.wait_for_service():
            pass

        request = GetEntityState.Request()
        request.name = name
        return self.get_entity_state_client.call_async(request)
    

    def set_entity_state(self, name, reference_frame, pose):

        while not self.set_entity_state_client.wait_for_service():
            pass

        state = EntityState()
        state.name = name
        state.pose = pose
        state.twist = Twist()
        state.reference_frame = reference_frame

        request = SetEntityState.Request()
        request.state = state
        
        return self.set_entity_state_client.call_async(request)
    
    # Initialize arena with items
    def initialize_arena(self):

        return False
    
    async def control_loop(self):

        if self.first_run:
            self.first_run = False

            for i in range(6):

                valid = False
                colour = random.choice(list(Colour))

                while not valid:

                    count = 0

                    for cluster in self.clusters.values():
                        if colour == cluster.colour:
                            count += 1
                
                    if count < 2:
                        valid = True
                    else:
                        colour = random.choice(list(Colour))

                cluster_id = "cluster" + str(self.cluster_counter)
                self.cluster_counter += 1

                x, y = self.generate_cluster_location(colour)

                self.clusters[cluster_id] = Cluster(x, y, colour)

                for j in range(random.randint(3, 5)):

                    x, y = self.generate_item_position(cluster_id)

                    item_id = "item" + str(self.item_counter)
                    self.items[item_id] = Item(x, y, colour, cluster_id)

                    self.get_logger().info(f'Spawning {item_id} of {colour} at ({x:.2f}, {y:.2f})')

                    await self.spawn_item(item_id, x, y, colour)

        await self.spawn_item("ready", 0.0, 0.0, Colour.RED, z=-0.5)

        model_list_msg = await self.get_model_list()

        # Update position of robots.

        for model_name in model_list_msg.model_names:
            if "robot" in model_name:
                if model_name not in self.robots:
                    self.robots[model_name] = Robot(0, 0)

        for robot_id, robot in self.robots.items():

            entity_state_msg = await self.get_entity_state(robot_id)
            robot_position = entity_state_msg.state.pose.position

            robot.x = round(robot_position.x, 2)
            robot.y = round(robot_position.y, 2)

            for item_id, item in self.items.items():

                # If item is that held by robot continue loop...
                if item_id is robot.item_held:
                    continue

            if robot.item_held is not None:
                # Update stored item position using robot's position
                self.items[robot.item_held].x = robot.x
                self.items[robot.item_held].y = robot.y

                # self.get_logger().info(f'Item {robot.item_held} at ({self.items[robot.item_held].x}, {self.items[robot.item_held].y})')

                # Update item location in simulation relative to robot frame,
                # in this case we hold it slightly above (z+0.15) the robot.
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.15

                await self.set_entity_state(robot.item_held, robot_id, pose)

        # Publish item holders
        item_holders = ItemHolders()

        for robot_id, robot in self.robots.items():
            
            item_holder = ItemHolder()
            item_holder.robot_id = robot_id

            if robot.item_held is None:
                item_holder.holding_item = False
                item_holder.item_colour = ""
                item_holder.item_value = 0
            else:
                item_holder.holding_item = True
                item_holder.item_colour = self.items[robot.item_held].colour.name
                item_holder.item_value = self.item_values[self.items[robot.item_held].colour]

            item_holders.data.append(item_holder)

        self.item_holders_publisher.publish(item_holders)

        # Publish item log
        item_log = ItemLog()

        # Use zones to publish number of coloured items retrieved
        for colour in Colour:
            match colour:
                case Colour.RED:
                    for _, zone in self.zones.items():
                        item_log.red_count += zone.items_returned[colour]
                case Colour.GREEN:
                    for _, zone in self.zones.items():
                        item_log.green_count += zone.items_returned[colour]
                case Colour.BLUE:
                    for _, zone in self.zones.items():
                        item_log.blue_count += zone.items_returned[colour]
                case _:
                    pass

        item_log.total_count = item_log.red_count + item_log.green_count + item_log.blue_count

        item_log.red_value = item_log.red_count * self.item_values[Colour.RED]
        item_log.green_value = item_log.green_count * self.item_values[Colour.GREEN]
        item_log.blue_value = item_log.blue_count * self.item_values[Colour.BLUE]
        item_log.total_value = item_log.red_value + item_log.green_value + item_log.blue_value

        self.item_log_publisher.publish(item_log)

def main(args=sys.argv):

    rclpy.init(args=args)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    node = ItemManager(args_without_ros)

    if multiprocessing.cpu_count() >= 3:
        executor = MultiThreadedExecutor()
    else:
        executor = MultiThreadedExecutor(num_threads=3)
    
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt: # TODO: Why does this sometimes take 2x Ctrl+c to kill?
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()