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
from std_msgs.msg import Float32MultiArray
import numpy as np
from ur5e_fe.msg import array2d


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

    def gui(self):
        #sg.theme('DarkAmber')

        izdelki = ['mleko', 'sok', 'kava']
        seznam_izdelkov = ""
        for i in izdelki:
            if len(seznam_izdelkov) == 0:
                seznam_izdelkov += i
            else:
                seznam_izdelkov += ", " + i

        layout = [[sg.Text('Izdelki, ki so na voljo: ' + seznam_izdelkov)],
                [sg.Text('Izberi izdelek'), sg.InputText()],
                [sg.Text('Vnesi kolicino'), sg.InputText()],
                [sg.Button('Oddaj narocilo'), sg.Button('Preklici narocilo')]]

        window = sg.Window('Oddaja narocila', layout)

        while True:
            event, values = window.read()
            #if event == sg.WIN_CLOSED or event == 'Preklici narocilo':
            if event == 'Preklici narocilo':
                break
            elif event == 'Oddaj narocilo':
                izdelek = values[0]
                kolicina = values[1]
                izdelek_OK = False
                kolicina_OK = True
                if izdelek in izdelki:
                    izdelek_OK = True
                else:
                    izdelek_OK = False
                    print('Izbrani izdelek zal ne obstaja.')
                if not kolicina.isdigit():
                    kolicina_OK = False
                    print('Prosim, zapisite kolicino s stevilko.')
                else:
                    kolicina_OK = True
                if izdelek_OK and kolicina_OK:
                    print('Narocilo oddano. Podatki o narocilu: ' + str(kolicina) + 'x ' + str(izdelek))
                    break

        window.close()
        return(izdelek, int(kolicina))

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

    def poz_umik(self):
        joint_goal = self.move_group.get_current_joint_values()
        #joint_goal = [1.186, -1.600, -1.006, -2.106, 1.570, 1.186]
        joint_goal = [1.186, -1.648, -0.864, -2.2, 1.57, 1.186]
        self.moveJ(joint_goal)

    def poz_pred_izdelki(self):
        joint_goal = self.move_group.get_current_joint_values()
        #joint_goal = [1.206, -1.806, -1.713, 0.387, 0.424, -0.061]
        joint_goal = [1.206, -1.837, -1.762, 0.467, 0.424, -0.061]
        self.moveL(joint_goal)
    
    def poz_pred_sokom(self):
        joint_goal = self.move_group.get_current_joint_values()
        #joint_goal = [1.042, -1.54, -2.011, 0.417, 0.588, -0.059]
        joint_goal = [1.042, -1.576, -2.065, 0.507, 0.588, -0.059]
        self.moveL(joint_goal)
    
    def poz_pobiranje(self, kolicina):
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.x -= (0.1 + ((kolicina-1) * 0.08))
        self.moveL(x_r)
    
    def prijemalo(self, p, s):
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io_2 = SetIORequest()
        set_io_2.fun = 1.0
        set_io_2.pin = p
        set_io_2.state = s
        result = set_io(set_io_2)
        rospy.sleep(0.1)

    def poz_po_pobiranju(self, kolicina):
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.x += (0.2 + ((kolicina-1) * 0.08))
        x_r.pose.position.z += 0.05
        self.moveL(x_r)

    def preverjanje_pobiranja(self, pin):
        x = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates, timeout=1)
        #print(x.digital_in_states[16].state)
        return x.digital_in_states[pin].state
    
    def spuscanje_izdelka(self):
        self.prijemalo(16, 1.0)
        rospy.sleep(0.01)
        self.prijemalo(17, 0.0)
        self.prijemalo(16, 0.0)
        rospy.sleep(0.1)

    def poz_ponovno_pobiranje(self):
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.x -= 0.01
        self.moveL(x_r)

    def poz_nad_paleto(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal = [1.499, -1.456, -1.486, -1.765, 1.575, -0.036]
        self.moveL(joint_goal)
    
    def poz_odlaganje_na_paleto(self, mesto, visina):
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.x += ((mesto-1) * 0.14)
        self.moveL(x_r)
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.z -= (0.3 - visina)
        self.moveL(x_r)

    def poz_po_odlaganju_na_paleto(self, mesto):
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.z += 0.23
        self.moveL(x_r)

    def poz_za_sliko(self):
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.z += 0.03
        self.moveL(x_r)

    def poz_pred_pobiranjem(self, aruco_id, aruco_cor, izdelek):
        id_izdelkov = {'mleko': 3.0, 'sok': 4.0, 'kava': 5.0}
        idx = -1
        for i in range(len(aruco_id)):
            if aruco_id[i] == id_izdelkov.get(izdelek):
                idx = i
        pozicija = aruco_cor[idx]
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.y = - pozicija / 1000.0
        #x_r.pose.position.y = -0.2
        self.moveL(x_r)

    def pobiranje_kave(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[5] += pi/2
        self.moveJ(joint_goal)
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.z -= 0.03
        self.moveL(x_r)

    
    def po_izdelek(self):
        #print("Grem domov.")
        #self.goHome()
        #print("Grem stran.")
        self.poz_umik()
        #self.poz_za_sliko()
        #print(self.izpis_pozicije())
        print("Slikam...")
        izdelek, kolicina = self.gui()
        count_izdelka = 1
        #vprasaj Bineta za lokacijo
        #aruco_id = rospy.wait_for_message('/aruco_location_id', Float32MultiArray, timeout=5)
        #aruco_cor = rospy.wait_for_message('/aruco_location_cor', Float32MultiArray, timeout=5)
        aruco_sub = rospy.wait_for_message('/aruco_location', array2d, timeout=5)
        #print(aruco_id)
        #print(aruco_cor)
        while count_izdelka <= kolicina:
            self.poz_pred_izdelki()
            self.poz_pred_pobiranjem(aruco_sub.index, aruco_sub.position, izdelek)
            if izdelek == 'kava':
                self.pobiranje_kave()
            self.poz_pobiranje(count_izdelka)
            print("Pobiram izdelek.")
            self.prijemalo(17, 1.0)
            rospy.sleep(1)
            counter = 0
            while not self.preverjanje_pobiranja(17) and counter < 3:
                print("Nimam izdelka :(")
                counter += 1
                self.poz_ponovno_pobiranje()
                rospy.sleep(1)
            if counter < 3:
                print("Imam izdelek :)")
            else:
                print("Zal mi ni uspelo pobrati izdelka :(")
            self.poz_po_pobiranju(count_izdelka)
            if counter < 3:
                print("Odlagam izdelek.")
                self.poz_umik()
                self.poz_nad_paleto()
                if izdelek == 'kava':
                    self.poz_odlaganje_na_paleto(count_izdelka, 0.09)
                else:
                    self.poz_odlaganje_na_paleto(count_izdelka, 0.07)
                self.spuscanje_izdelka()
                print("Sem odlozil izdelek.")
                self.poz_po_odlaganju_na_paleto(count_izdelka)
            else:
                self.spuscanje_izdelka()
            count_izdelka += 1
        self.poz_umik()
        
        


    
    def izpis_pozicije(self):
        joint_goal = self.move_group.get_current_joint_values()
        print("Trenutna pozicija:")
        print([float(decimal.Decimal("%.3f" % e)) for e in joint_goal])
        
        

if __name__ == '__main__':
    rospy.init_node('moveit_ur5e_test', anonymous=True)
    
    ur5e_moveit_obj = ur5e_moveit()
    try:
        #ur5e_moveit_obj.get_basic_info()
        #ur5e_moveit_obj.goHome()
        ur5e_moveit_obj.po_izdelek()
        #ur5e_moveit_obj.poz_umik()
        #ur5e_moveit_obj.izpis_pozicije()
    except rospy.ROSInterruptException:
        pass

