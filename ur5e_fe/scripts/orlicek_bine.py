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
from ur_msgs.msg import IOStates
import decimal
import PySimpleGUI27 as sg
import cv2 as cv
#from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ur5e_fe.msg import array2d

    

def callback(data):
    pub_id = rospy.Publisher('/aruco_location_id', Float32MultiArray, queue_size=10)
    pub_cor = rospy.Publisher('/aruco_location_cor', Float32MultiArray, queue_size=10)
    pub = rospy.Publisher('/aruco_location', array2d, queue_size=10)
    podatki_za_izvoz_id = Float32MultiArray()
    podatki_za_izvoz_cor = Float32MultiArray()
    podatki_za_izvoz = array2d()
    br = CvBridge()
    img = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

    if ids is not None:
        podatki_za_izvoz_id.data = ids
        #podatki_za_izvoz_cor.data = corners
        podatki_za_izvoz_cor.data = []
        pub_id.publish(podatki_za_izvoz_id)
        for i in range(len(ids)):
            x = corners[i][0][0]
            #print(str(i) + ": " + str(x))
            #print(x[0])
            #print("Zivjo, tukaj Bine!")
            #print(x[0])
            podatki_za_izvoz_cor.data.append(624.5 - 1.403 * x[0] - 30)
            #print(podatki_za_izvoz_cor.data)
            podatki_za_izvoz.index.append(ids[i][0])
            podatki_za_izvoz.position.append(624.5 - 1.403 * x[0] - 30)
        #print(podatki_za_izvoz_id.data)
        #print(podatki_za_izvoz_cor.data)
        pub_cor.publish(podatki_za_izvoz_cor)
        pub.publish(podatki_za_izvoz)

    cv.aruco.drawDetectedMarkers(img, corners, ids, (255, 0, 0))

    cv.imshow("camera", img)
    cv.waitKey(1)
    
def receive_message():
    rospy.init_node('camera_sub', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    rospy.spin()
    cv.destroyAllWindows()

        
        

if __name__ == '__main__':
    try:
        receive_message()
    except rospy.ROSInterruptException:
        pass

