Week 4
------
Solutions for Week 4's practicals are available in this folder. 

There are two launch files:
* week_4_launch.py: provides the simulation environment, which can be used with the node robot_controller,
                    as implemented in [robot_controller.py](week_4/robot_controller.py), 
                    or teleoperation of the turtlebot.
* week_4_rc_launch.py: in addition to setting up the simulation environment in Gazebo, it runs
                    two nodes rc_driving and rc_obstacle_detector, as implemented in
                    [rc_driving.py](week_4/rc_driving.py) and 
                    [rc_obstacle_detector.py](week_4/rc_obstacle_detector.py). 
                    This implementation is designed to illustrate how the robochart model 
                    [turtlebot-robochart-move-composite](robochart/turtlebot-robochart-move-composite/) may
                    be implemented in Python.