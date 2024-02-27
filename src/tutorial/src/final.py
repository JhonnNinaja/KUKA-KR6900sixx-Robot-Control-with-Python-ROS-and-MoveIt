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

    def cartesian_paths(self):
        waypoints = []
        scale = 1
        self.wpose = self.move_group.get_current_pose().pose

     
        self.wpose.position.y += scale * 0.2  

        self.aux_y = self.wpose.position.y
        self.aux_z = self.wpose.position.z
        
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

    def stop(self):
        flag = True
        while flag:
            print("en Y: "+str(self.wpose.position.y))
            print("en z: "+str(self.wpose.position.z))
            if  (self.wpose.position.y < 0.4 and self.wpose.position.y > -0.4)  and   (self.wpose.position.z < 0.95 and self.wpose.position.z > -0.88):
                print("flag2")
                flag = False
    def execute_plan(self):
        self.move_robot()
        contador = 0
        while not self.ctrl_c:
            #self.move_robot()
            self.move_group.execute(self.plan, wait=True)
            self.stop()
            contador += 1
            print("contador: "+ str(contador))
          


        

    
    def shutdownhook(self):
        self.ctrl_c = True


if __name__ == '__main__':
    rospy.init_node('tutorial', anonymous=True)
    rosbot_object = Tutorial1()

    try:
  
        input("Press enter to continue")
        rosbot_object.cartesian_paths() #cartesianos
        input("Press enter to execute plan")
        rosbot_object.execute_plan()
    except rospy.ROSInterruptException:
        pass
