

**Project: KUKA KR6900sixx Robot Control with Python, ROS, and MoveIt!**

This project demonstrates how to control a KUKA KR6900sixx industrial robot using Python, ROS (Robot Operating System), and the MoveIt! motion planning framework. 


## KUKA KR6900sixx Overview

The KUKA KR6900sixx is a versatile industrial robot designed for a wide range of applications. Key features include:

* **High Speed:** Optimized for fast movements and short cycle times.
* **Precision:** Delivers excellent accuracy and repeatability.
* **Integrated Power Supply:** Compact design with internal power supply components.
* **Flexible Mounting:** Can be mounted on the floor, ceiling, or wall.
* **Low Maintenance:** Designed for minimal maintenance requirements.
* **Large Workspace:** Capable of reaching a wide range of positions.

## Code Description

The core of this project is the `test3.py` Python file. Here's an explanation of the key components:

**Initialization**

1. **ROS Node:** The code initializes a ROS node, which is essential for communication within the ROS ecosystem.
2. **MoveIt! Commander:** Initializes the `moveit_commander` interface, allowing control and planning with MoveIt!.
3. **Planning Group:** Defines the `end_effector` planning group, which represents the part of the robot we want to control (i.e., the end of the arm).
4. **Trajectory Publisher:** Initializes a publisher for visualizing planned robot trajectories in Rviz.

**Movement Methods**

* **`all_close()`**
   * Compares two robot states (current and goal) and recursively reduces the difference between them.
   * Used to ensure that the robot reaches its intended target position.
* **`get_current_state()`** 
   * Gets the robot's current joint positions.
   * Used to determine the robot's starting point and update its state during movement.
* **`cartesian_paths()`**
   * Defines a Cartesian path (a series of waypoints in 3D space) for the end-effector to follow.
   * One of the most common motion planning methods due to its intuitive specification of paths.

**Plan Execution**

* Inside the `while` loop:
   * **Plan:** Calculate a motion plan based on the desired movement type (Cartesian, quaternion, etc.).
   * **Execute:** Command the robot to execute the planned trajectory.
   * **Reset:** Prepare the robot for the next movement cycle.

**Specific Command: `self.quaternions_paths(0, 0, 0)`**

* Moves the robot's end-effector to a specific position in space.
* Parameters:
   * `x`: Displacement along the x-axis (meters)
   * `y`: Displacement along the y-axis (meters)
   * `z`: Displacement along the z-axis (meters)
* In this case, the robot moves -0.2 meters along the x-axis, and -0.2 meters along the z-axis, with maximum velocity.

## URDF Files

The URDF (Unified Robot Description Format) files in the `urdf` directory define the robot's physical and kinematic properties:

* **kr6900sixx\_macro.xacro:** The main URDF file containing:
   * **Visuals:** Links to 3D models (`.dae` files) for visualization.
   * **Collisions:** Links to simplified geometry (`.stl` files) for collision detection.
   * **Joints:** Defines joint types (revolute, prismatic), limits, and their connections.

## Requirements

* Python 3
* ROS (Robot Operating System)
* MoveIt!
* rospy
* moveit_commander
* moveit_msgs
* geometry_msgs
* numpy

## Usage

1. **Start ROS:** `roscore`
2. **Run the code:** `rosrun package_name test3.py` 

