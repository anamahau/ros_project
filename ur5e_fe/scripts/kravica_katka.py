#! /usr/bin/python

from colorsys import rgb_to_yiq
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from math import pi
import math
#from moveit_commander.conversions import pose_to_list
import tf
from numpy import linalg as LA
from dirkin_UR5e import dirkinUR5e
from ur_msgs.srv import SetIO, SetIORequest


class ur5e_moveit():
    
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)

        #set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
       
        # robots kinematic model and the robots current joint states
        self.robot = moveit_commander.RobotCommander()
        self.ur5e_dirkin = dirkinUR5e()
                
        # remote interface for getting, setting, and updating the robots internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # interface to a planning group (group of joints).
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name,robot_description = "/robot_description")
        self.move_group.set_pose_reference_frame('base_link_inertia')
        self.move_group.set_end_effector_link('tool0')
         
        self.move_group.clear_pose_targets()

        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True


    def get_basic_info(self):

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print( "============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
        
    def axisangle_to_quaternion(self, axan):
        
        angle = LA.norm(axan)
        ax = axan/angle
                
        q = Quaternion()
        q.x = ax[0] * math.sin(angle/2)
        q.y = ax[1] * math.sin(angle/2)
        q.z = ax[2] * math.sin(angle/2)     
        q.w = math.cos(angle/2)
        
        return q

    def moveJ(self, goal):

        self.move_group.set_planner_id('PTP')
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        #self.move_group.set_pose_reference_frame('base_link_inertia')
        
        # plan and execute
        self.move_group.go(goal, wait=True)        
        

    def moveL(self, goal):

        self.move_group.set_planner_id('LIN')
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        #self.move_group.set_pose_reference_frame('base_link_inertia')
        
        self.move_group.clear_pose_targets()
        
        # plan and execute
        self.move_group.go(goal, wait=True)


    def goHome(self):

        #print("goHome funkcija")
        joint_goal = self.move_group.get_current_joint_values()
        #print(joint_goal)
        joint_goal[0] = 0
        joint_goal[1] = -pi/2
        joint_goal[2] = -pi/2
        joint_goal[3] = -pi/2
        joint_goal[4] = pi/2
        joint_goal[5] = 0

        # move to home position
        self.moveJ(joint_goal)

    def po_mleko(self):
        print("Grem domov.")
        self.goHome()

        print("Grem po mleko.")
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.z += 0.2
        x_r.pose.position.y -= 0.4
        self.moveL(x_r)
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.x += 0.1
        self.moveL(x_r)
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.y -= 0.1
        x_r.pose.position.x += 0.1
        self.moveL(x_r)
        x_r_2 = self.move_group.get_current_pose()

        print("Plesem.")
        point_1 = PoseStamped()
        point_1.header.seq = 0
        point_1.header.stamp = rospy.Time.now()
        point_1.header.frame_id = "base_link_inertia"
        x_r = self.move_group.get_current_pose()
        point_1.pose.position.x = 0.389
        point_1.pose.position.y = 0.391
        point_1.pose.position.z = 0.531
        ax = 2.555
        ay = -2.281
        az = 2.423
        axl = [ax, ay, az]
        quat = self.axisangle_to_quaternion(axl)
        point_1.pose.orientation.x = quat.x
        point_1.pose.orientation.y = quat.y
        point_1.pose.orientation.z = quat.z
        point_1.pose.orientation.w = quat.w
        self.moveL(point_1)

        print("Sedaj grem pa res po mleko.")
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.x -= 0.1
        self.moveL(x_r)

        raw_input("Pritisni enter za pobiranje mleka!")

        print("Pobiram mleko.")
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io_2 = SetIORequest()
        set_io_2.fun = 1.0
        set_io_2.pin = 17
        set_io_2.state = 1.0
        result = set_io(set_io_2)

        print("Imam mleko :)")
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.x += 0.1
        x_r.pose.position.z += 0.05
        self.moveL(x_r)

        print("Oglagam mleko.")
        

if __name__ == '__main__':
    rospy.init_node('moveit_ur5e_test', anonymous=True)
    
    ur5e_moveit_obj = ur5e_moveit()
    try:
        #ur5e_moveit_obj.get_basic_info()
        #ur5e_moveit_obj.goHome()
        ur5e_moveit_obj.po_mleko()
    except rospy.ROSInterruptException:
        pass

