#! /usr/bin/python

from colorsys import rgb_to_yiq
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from math import pi
#from moveit_commander.conversions import pose_to_list
import tf
from numpy import linalg as LA
from dirkin_UR5e import dirkinUR5e
from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ur_msgs.srv import SetSpeedSliderFraction, SetSpeedSliderFractionRequest
from ur_msgs.msg import IOStates



def sick_scanner(data):
    set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
    set_speed_2 = SetSpeedSliderFractionRequest()
    knof = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates, timeout=1)
    stanje_knof = knof.digital_in_states[0].state
    x = data.ranges[810:1350]
    min_x = min(x)
    if stanje_knof:
        set_speed_2.speed_slider_fraction = 0.8
    else:
        if min_x < 0.2:
            set_speed_2.speed_slider_fraction = 0.05
        else:
            set_speed_2.speed_slider_fraction = 0.4
    set_speed(set_speed_2)
    #print(neki2)
    #print(set_speed_2.speed_slider_fraction)
    
def receive_message():
    rospy.init_node('camera_sub', anonymous=True)
    #rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    rospy.Subscriber('/sick_safetyscanners/scan', LaserScan, sick_scanner)
    rospy.spin()

        
        

if __name__ == '__main__':
    try:
        receive_message()
    except rospy.ROSInterruptException:
        pass

