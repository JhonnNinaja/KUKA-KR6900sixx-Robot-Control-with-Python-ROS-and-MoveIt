#!/usr/bin/env python3
# Python 2/3 compatibility imports
from __future__ import print_function

#from six.moves import input


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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

   
    def move_robot(self):
        rospy.sleep(1)
   
        
        joint_a = self.move_group.get_current_joint_values()
        joint_a[0] = 0
        joint_a[1] = -1.57
        joint_a[2] = 1.57
        joint_a[3] = 2
        joint_a[4] = 1.57
        joint_a[5] = 2


            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_a, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
    def move_robot2(self,x_goal,y_goal,z_goal):
        x, y, z, qx, qy, qz, qw = pose_to_list(self.move_group.get_current_pose().pose) # this line obtains the actual pose and quaternions of final effector
  
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw
        pose_goal.position.x = x+x_goal
        pose_goal.position.y = y+y_goal
        pose_goal.position.z = z+z_goal

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
        wpose = self.move_group.get_current_pose().pose

     
        wpose.position.y += scale * 0.2  
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * 0.2  #
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.2  
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += scale * 0.2  #
        waypoints.append(copy.deepcopy(wpose))



        

      
        (self.plan, self.fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold



    
    def execute_plan(self):
        while not self.ctrl_c:
            
            self.move_group.execute(self.plan, wait=False)
            rospy.sleep(14)


        

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        
        box_name = self.box_name
        scene = self.scene        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL
    def add_box(self):
        box_name = self.box_name
        scene = self.scene

        timeout = 4
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.175, 0.175,0.175))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names
        grasping_group = "end_effector"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=box_name)
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )
    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )
    def shutdownhook(self):
        self.ctrl_c = True


if __name__ == '__main__':
    rospy.init_node('tutorial', anonymous=True)
    rosbot_object = Tutorial1()

    try:
        #rosbot_object.move_robot()
        input("Press enter to set queternions path")
        rosbot_object.move_robot2()
        #input("Press enter to continue")
        #rosbot_object.cartesian_paths() #cartesianos
        #input("Press enter to display plan")
        #rosbot_object.display_plan()
        #input("Press enter to execute plan")
        #rosbot_object.execute_plan()
        #input("Press enter to add box")
        #rosbot_object.add_box()
        #input("Press enter to attach box")
        #rosbot_object.attach_box()
        #input("Press enter to detach box")
        #rosbot_object.detach_box()
        #input("Press enter to remove box")
        #rosbot_object.remove_box
    except rospy.ROSInterruptException:
        pass
