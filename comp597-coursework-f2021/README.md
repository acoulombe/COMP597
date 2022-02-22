## McGill University - COMP 597 - Applied Robotics coursework
The code in this repository is a cumulation of robotic controllers produced as coursework for the Applied Robotics course at McGill. The produced code is found under `comp597-coursework-f2021/`

# Assignment 1
Keyboard teleoperation on a husky robot from Clearpath in simulation. For simulation, Gazebo is used. The controller takes a keyboard input, converts it to a robot twist command and provided to the husky robot to execute.

Required packages:
- husky
- keyboard_reader
- husky_teleop_controller

# Assignment 2
Trajectory planner for a Firefly drone using cubic polynomial to generate the trajectory using a start and goal state for a given time period. For simulation, Gazebo is used. The planner provides the state to the drone position controller that tracks the planned trajectory.

Required packages:
- rotors_simulator
- mav_comm
- cubic_polynomial_planner

# Assignment 3
Implementation of an inverse kinematics controller with
redundancy resolution. The trajectory is planned for the end-effector positions of the robot using
a cubic polynomial planner. The final output of the controller is the joint positions of the robot. This system is tested on a Kinova arm in Gazebo.

Required packages:
- kinova_ros
- inverse_kinematic_controller

# Assignment 4
Implementation of an inverse dynamics controller with
redundancy resolution. The trajectory is planned for the end-effector positions of the robot using
a potential field planner. The final output of the controller is the joint torques of the robot. This system is tested on a Kinova arm in Gazebo.

Required packages:
- kinova_ros
- robot_model
- pinocchio
- inverse_dynamics_controller

# Assignment 5
Implementation of a pick and place task to grasp an object. The implementation can use either an inverse kinematics controller or an inverse dynamics controller to control the Kinova arm. The motion is planned with a potential field planner. The final output of the controller to the robot depends on the controller: joint positions for inverse kinematics and joint torques for inverse dynamics. The simulation of the task is in Gazebo where the object to grab is a small sphere.

Required packages:
- kinova_ros
- robot_model
- pinocchio
- highlevel_controller

# Assignment 6
Implementation of an autonomous navigation system for a husky robot from Clearpath. The robot is equipped with a LIDAR for getting environmental information. The goal of the planner is to bring the robot to a goal location in various maps with no prior knowledge of the environments. The environments are all populated with various obstacles, static and dynamic, and with varying terrain, flat and uneven. A few planners were implemented, namely the Bug Algorithm, the A* Algorithm for finite horizon and a planner with custom modification to the Bug Algorithm for better decision making in the environment. The reactive planner provides the next twist control to the robot base to track.

Required packages:
- cpr_gazebo
- husky
- base_controller

Environments:
- empty
- agriculture
- cafe
- mud
- office
- orchard
- inspection