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
#   * Pedro Ribeiro - initial implementation
#
# Implementation of RoboChart controller 'Driving' with single Finite State machine
# 'TurtleBot3FSMRC' as in the project in 'robochart/turtlebot-robochart-move-composite'.
 
import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from auro_interfaces.msg import StringWithPose, Item, ItemList, Direction

from tf_transformations import euler_from_quaternion
import angles

from enum import Enum
import random
import math

LINEAR_VELOCITY  = 0.3 # Metres per second
ANGULAR_VELOCITY = 0.25 # Radians per second

# For keeping track of action execution within a state
class Action(Enum):
    ENTRY = 0
    INTERRUPTABLE = 1
    BUSY = 2
    EXIT = 3

# Class for handling obstacle event as an input
class Obstacle:

    def __init__(self):
        self.direction = None
        self.triggered = False

    def trigger(self, direction):
        self.direction = direction
        self.triggered = True
    
    def reset(self):
        self.triggered = False

# Class for handling RoboChart clocks
class Clock:

    def __init__(self, time_unit, timer_period):
        self.counter = 0
        self.time_unit = time_unit
        self.timer_period = timer_period

    def step(self):
        self.counter = self.counter + 1

    def reset(self):
        self.counter = 0

    def since(self):
        return self.timer_period * self.counter / self.time_unit

# Finite state machine (FSM) including its states
class TurtleBot3FSMRC:

    LVEL = LINEAR_VELOCITY
    AVEL = ANGULAR_VELOCITY

    def __init__(self, logger, move, time_unit, timer_period):
        self.clock_C = Clock(time_unit, timer_period)
        self.move = move
        self.logger = logger

        self.obstacle = {}
        self.obs = {}

        self.action = Action.ENTRY
        self.state = self.State.INITIAL
        self.state_driving = self.Driving(self.LVEL, self.AVEL, self.clock_C, self.move, logger)
        self.state_rotating = self.Rotating(logger)

    def __repr__(self):
        match self.state:
            case self.State.INITIAL:
                return '(State: Initial)'
            case self.State.DRIVING:
                return f'(State: {self.state_driving})'
            case self.State.ROTATING:
                return f'(State: {self.state_rotating})'
            case _:
                return "<empty>"

    class State(Enum):
        '''Enumeration for keeping track of current state'''
        INITIAL = 0
        DRIVING = 1
        ROTATING = 2

    def step_clocks(self):
        '''Steps all clocks by one control cycle of timer_period'''
        self.clock_C.step()

    def execute(self, obstacle):
        '''Executes state machine given an obstacle object'''
        self.obstacle = obstacle
        return self.execute_stm()

    def execute_stm(self):
        '''Execution of state machine depending on current value of self.state'''
        match self.state:
            case self.State.INITIAL:
                self.clock_C.reset()
                self.state = self.State.DRIVING
                self.action = Action.ENTRY
                return True
            
            case self.State.DRIVING:
                 match self.action:
                     
                    case Action.ENTRY:
                        # No entry actions, therefore attempt to move to interruptable action state as soon as possible.
                        exec = self.state_driving.execute()
                        if self.state_driving.interruptable():
                            self.action = Action.INTERRUPTABLE
                            return True
                        
                        return exec
                            
                    case Action.INTERRUPTABLE:
                        exec = self.state_driving.execute()
                        if not self.state_driving.interruptable():
                            self.action = Action.BUSY
                            return True
                        
                        # We give priority to inner state's execution over our own transitions.
                        if exec:
                            return exec
                        else:    
                            # No further work to do in inner state, evaluate our own transitions.
                            if not self.obstacle is None and self.obstacle.triggered:
                                
                                # We reset the 'event', so that is not handled again before it is triggered again by ROS
                                self.obstacle.reset() 
                                self.obs = self.obstacle.direction

                                # We interrupt the state
                                self.state_driving.interrupt()

                                self.logger.info(f"Obstacle {self.obs}")
                                # Then junction:
                                if self.obs == Direction.LEFT or self.obs == Direction.FRONT:
                                    #self.logger.info("LEFT or FRONT")
                                    self.move(0.0, -self.AVEL)
                                elif self.obs == Direction.RIGHT or self.obs == Direction.FRONT:
                                    #self.logger.info("RIGHT or FRONT")
                                    self.move(0.0, self.AVEL)

                                self.logger.info("Transition from 'DRIVING' to 'ROTATING' ")
                                self.clock_C.reset()
                                self.state = self.State.ROTATING
                                self.action = Action.ENTRY
                                return True

                            return False

                    case Action.BUSY:

                        # We cannot interrupt inner state
                        exec = self.state_driving.execute()
                        if self.state_driving.interruptable():
                            self.action = Action.INTERRUPTABLE
                            return True
                        else:
                            return exec

                    case _:
                         return False
                    
            case self.State.ROTATING:
                match self.action:
                     
                    case Action.ENTRY:

                        # No entry actions, therefore attempt to move to interruptable action state as soon as possible.
                        exec = self.state_rotating.execute()
                        if self.state_rotating.interruptable():
                            self.action = Action.INTERRUPTABLE
                            return True
                        
                        return exec
                            
                    case Action.INTERRUPTABLE:

                        exec = self.state_rotating.execute()
                        if not self.state_rotating.interruptable():
                            self.action = Action.BUSY
                            return True
                        
                        # We give priority to inner state's execution over our own transitions.
                        if exec:
                            return exec
                        else:    
                            # No further work to do in inner state, evaluate our own transitions.
                            if self.clock_C.since() >= (math.pi/5)/self.AVEL:
                                self.state_rotating.interrupt()
                                self.logger.info("Transition from 'ROTATING' to 'DRIVING' ")
                                self.state = self.State.DRIVING
                                self.action = Action.ENTRY
                                return True

                            return False

                    case Action.BUSY:

                        # We cannot interrupt inner state
                        exec = self.state_rotating.execute()
                        if self.state_rotating.interruptable():
                            self.action = Action.INTERRUPTABLE
                            return True
                        else:
                            return exec

                    case _:
                         return False

            case _:
                pass

    # A composite state
    class Driving:
        
        class State(Enum):
            INITIAL = 0
            FORWARD = 1
            TURNING = 2
            COLLECTING = 3

        def __init__(self, lvel, avel, clock_C, move, logger):
            self.state = self.State.INITIAL
            self.action = Action.ENTRY
            self.LVEL = lvel
            self.AVEL = avel
            self.clock_C = clock_C
            self.move = move
            self.logger = logger

        def __repr__(self):
            return f'(Driving: {self.state}, action: {self.action})'
        
        def execute(self):
            return self.execute_stm()

        def interruptable(self):
            return self.action == Action.INTERRUPTABLE
        
        def interrupt(self):
            self.state = self.State.INITIAL
            self.action = Action.ENTRY

        def execute_stm(self):
            match self.state:
                case self.State.INITIAL:
                    self.clock_C.reset()
                    self.state = self.State.FORWARD
                    return True

                case self.State.FORWARD:
                    match self.action:
                        case Action.ENTRY:
                            self.move(self.LVEL, 0.0)
                            self.action = Action.INTERRUPTABLE
                            return True
                        case Action.INTERRUPTABLE:
                            if self.clock_C.since() > 5:
                                self.logger.info("Transition from 'FORWARD' to 'TURNING' ")
                                self.clock_C.reset()
                                self.action = Action.ENTRY
                                self.state = self.State.TURNING
                                return True
                            else:
                                return False # No more work to do.
                
                case self.State.TURNING:
                    match self.action:
                        case Action.ENTRY:
                            self.move(0.0, self.AVEL)
                            self.action = Action.INTERRUPTABLE
                            return True
                        case Action.INTERRUPTABLE:
                            if self.clock_C.since() > 1:
                                self.logger.info("Transition from 'TURNING' to 'FORWARD' ")
                                self.clock_C.reset()
                                self.action = Action.ENTRY
                                self.state = self.State.FORWARD
                                return True
                            else:
                                return False # No more work to do.

                case _:
                    return False
        
    # A simple state
    class Rotating:
        def __init__(self, logger):
            self.action = Action.ENTRY
            self.logger = logger

        def interruptable(self):
            return self.action == Action.INTERRUPTABLE
        
        def execute(self):
            return self.execute_stm()

        def interrupt(self):
            self.action = Action.ENTRY

        def execute_stm(self):
            match self.action:
                case Action.ENTRY:
                    self.action = Action.INTERRUPTABLE
                    return True
                case Action.INTERRUPTABLE:
                    return False
                case _:
                    return False

        def __repr__(self):
            return f'(Rotating: {self.action})'

class Driving(Node):

    def __init__(self):
        super().__init__('rc_driving')

        self.obstacle = Obstacle()
         
        self.pose = Pose() # Current pose (position and orientation), relative to the odom reference frame
        
        # Subscribes to Odometry messages published on /odom topic
        # http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        #
        # Final argument can either be an integer representing the history depth, or a Quality of Service (QoS) profile
        # https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py#L1335-L1338
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py#L1187-L1196
        #
        # If you only specify a history depth, rclpy defaults to QoSHistoryPolicy.KEEP_LAST
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos.py#L80-L83
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Subscription on the obstacle topic published to by the 'rc_obstacle_detector.py'.
        self.obstacle_subscriber = self.create_subscription(
            Direction,
            '/obstacle',
            self.obstacle_callback,
            10
        )
        
        # Publishes Twist messages (linear and angular velocities) on the /cmd_vel topic
        # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        # 
        # Gazebo ROS differential drive plugin subscribes to these messages, and converts them into left and right wheel speeds
        # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp#L537-L555
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publishes custom StringWithPose (see auro_interfaces/msg/StringWithPose.msg) messages on the /marker_input topic
        # The week3/rviz_text_marker node subscribes to these messages, and ouputs a Marker message on the /marker_output topic
        # ros2 run week_3 rviz_text_marker
        # This can be visualised in RViz: Add > By topic > /marker_output
        #
        # http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
        # http://wiki.ros.org/rviz/DisplayTypes/Marker
        self.marker_publisher = self.create_publisher(StringWithPose, '/marker_input', 10)

        # Creates a timer that calls the control_loop method repeatedly - each loop represents single iteration of the FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer_unit = 1 # 1s

        # Instantiation of the TurtleBot3FSMRC, where we pass, the ROS logger, the implementation of the move operation,
        # the timer unit and the timer period.
        self.STM = TurtleBot3FSMRC(self.get_logger(), self.move, self.timer_unit, self.timer_period)

        self.timer = self.create_timer(self.timer_period, self.control_loop)

    # Called every time odom_subscriber receives an Odometry message from the /odom topic
    #
    # The Gazebo ROS differential drive plugin generates these messages using kinematic equations, and publishes them
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp#L434-L535
    #
    # This plugin is configured with physical measurements of the TurtleBot3 in the SDF file that defines the robot model
    # https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble-devel/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf#L476-L507
    #
    # The pose estimates are expressed in a coordinate system relative to the starting pose of the robot
    def odom_callback(self, msg):
        self.pose = msg.pose.pose # Store the pose in a class variable

    # Whenever we receive an obstacle, we 'trigger' the event so it is available for the FSM.
    def obstacle_callback(self, msg):
        self.obstacle.trigger(msg.direction)

    # Implementation of move operation to be used by FSM.
    def move(self, lvel, avel):
        self.cmd_vel(Twist(linear=Vector3(x=lvel, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=avel)))

    # Actual implementation of cmd_vel that outputs on cmd_vel.
    def cmd_vel(self, twist):
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Output on cmd_vel: {twist}")

    # Control loop for the FSM - called periodically by self.timer
    def control_loop(self):

        # Send message to rviz_text_marker node
        marker_input = StringWithPose()
        marker_input.text = str(self.STM) # str(self.state) # Visualise robot state as an RViz marker
        marker_input.pose = self.pose # Set the pose of the RViz marker to track the robot's pose
        self.marker_publisher.publish(marker_input)

        # execute returns whether there is still work to do in the current time_period,
        # so execute it until no longer true.
        while self.STM.execute(self.obstacle):
            continue

        # step clocks once per call to control_loop every time_period.
        self.STM.step_clocks()

    def destroy_node(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
        super().destroy_node()

def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = Driving()

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