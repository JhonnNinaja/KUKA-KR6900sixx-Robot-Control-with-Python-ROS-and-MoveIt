#!/usr/bin/env python3
# Python 2/3 compatibility imports
from __future__ import print_function

#from six.moves import input

import math
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
class Tutorial1():
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        self.plan = 0
        self.fraction = 0
        self.box_name = ""
      
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "end_effector"

        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        self.eef_link = eef_link
        #------------ettetianl----------------#
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self.rate = rospy.Rate(10)
        self.initial_x = 0
        self.initial_y = 0
        self.initial_z = 0


    def all_close(self, goal, actual, tolerance):
    
        if type(goal) is list:
            print("esta entrando en el primero")
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
            # Euclidean distance
            d = dist((x1, y1, z1), (x0, y0, z0))
            # phi = angle between orientations
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

        return True
    def go_to_initial_state(self):
   
        
        joint_a = self.move_group.get_current_joint_values()
        joint_a[0] = 0
        joint_a[1] = -1.57
        joint_a[2] = 1.57
        joint_a[3] = 0
        joint_a[4] = 0
        joint_a[5] = 0


            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_a, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        
        return self.all_close(joint_a, current_joints, 0.01)
   

    def quaternions_paths(self,x_goal,y_goal,z_goal,set_velocity):
        x, y, z, qx, qy, qz, qw = pose_to_list(self.move_group.get_current_pose().pose) # this line obtains the actual pose and quaternions of final effector
        
        quaternion = quaternion_from_euler(0,3.1416,0) #roll, pitch, yaw (x,y,z) for previous to gripper, only for the final effector
        pose_goal = geometry_msgs.msg.Pose()

        
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = self.initial_x+x_goal
        pose_goal.position.y = self.initial_y+y_goal
        pose_goal.position.z = self.initial_z+z_goal
        print(x)
        print(y)
        print(z)
        
        print("--------------")
        self.move_group.set_max_velocity_scaling_factor(set_velocity)
        self.move_group.set_pose_target(pose_goal)
        
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
    def cartesian_paths(self):
        waypoints = []
        scale = 1
        self.wpose = self.move_group.get_current_pose().pose

     
        self.wpose.position.y += scale * 0.2  
        waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.z -= scale * 0.2  #
        waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.y -= scale * 0.2  
        waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.z += scale * 0.2  #
        waypoints.append(copy.deepcopy(self.wpose))

        (self.plan, self.fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

    
          
    def execute_plan(self):
        self.initial_x, self.initial_y, self.initial_z, qx, qy, qz, qw = pose_to_list(self.move_group.get_current_pose().pose)
        #self.quaternions_paths(0.3,0,0,1) #x,y,z
        self.quaternions_paths(-0.2,0,0,1) #x,y,z, 0-1
        
        #while not self.ctrl_c:
        
        self.quaternions_paths(-0.2,0.05,0,0.01) #x,y,z
        self.quaternions_paths(-0.2,0.1,0,0.01) #x,y,z
        self.quaternions_paths(-0.2,0.15,0,0.01) #x,y,z
        self.quaternions_paths(-0.2,0.2,0,0.01) #x,y,z
        self.quaternions_paths(-0.2,0.2,-0.2,1) #x,y,z
        self.quaternions_paths(-0.2,0,-0.2,1) #x,y,z
        self.quaternions_paths(-0.2,0,0,0.01) #x,y,z
        
        """
        self.quaternions_paths(0,0,0.05,0.01) #x,y,z,(0-1)
        self.quaternions_paths(0,0,0.05,0.01) #x,y,z
        self.quaternions_paths(0,0.05,-0.05,0.01) #x,y,z
        self.quaternions_paths(0,0.05,-0.05,0.01) #x,y,z
        self.quaternions_paths(0,0,0.05,0.01) #x,y,z
        self.quaternions_paths(0,0,0.05,0.01) #x,y,z

        self.quaternions_paths(0,0,-0.05,0.01) #x,y,z
        self.quaternions_paths(0,0,-0.05,0.01) #x,y,z
        self.quaternions_paths(0,-0.05,0.05,0.01) #x,y,z
        self.quaternions_paths(0,-0.05,0.05,0.01) #x,y,z
        self.quaternions_paths(0,0,-0.05,0.01) #x,y,
        self.quaternions_paths(0,0,-0.05,0.01) #x,y,
        """

        
        self.quaternions_paths(0.2,0,0,1) #x,y,z
    
    def shutdownhook(self):
        self.ctrl_c = True


if __name__ == '__main__':
    rospy.init_node('tutorial', anonymous=True)
    rosbot_object = Tutorial1()

    rosbot_object.go_to_initial_state()
    try:
        #rosbot_object.move_robot()
        #input("Press enter to continue")
        #rosbot_object.cartesian_paths() #cartesianos
        input("Press enter to execute plan")
        rosbot_object.execute_plan()
    except rospy.ROSInterruptException:
        pass
