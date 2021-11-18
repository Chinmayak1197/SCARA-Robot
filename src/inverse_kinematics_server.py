#!/usr/bin/env python

from foundations_assignment.srv import GetJointValues, GetJointValuesResponse
import rospy
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *


def handle_get_joint_values(GetJointValues):
    
    Pose = GetJointValues.pose
    position = Pose.position
    x_pos = position.x
    y_pos = position.y
    z_pos = position.z

    # Need to change the values of a1, a2 and a3.
    a0 = 0.1            #base offset
    a1 = 1
    a2 = 1
    a3 = 1

    # calculation for d3
    d3 = (a3+a0 - z_pos)

    # calculation for theta2
    A = (x_pos**2 + y_pos**2 - a1**2 - a2**2)/(2*a2*a2) 
    theta_2 = math.atan2(math.sqrt(1-A**2), A)
    
    # calculation for theta1
    theta1_x = a1 + a2*math.cos(theta_2)
    theta1_y = a2*math.sin(theta_2)
    theta_1 = np.arctan2(y_pos,x_pos) - np.arctan2(theta1_y,theta1_x)

    joint_names = ['theta_1','theta_2','d3']
    joint_pos = [theta_1, theta_2, d3]

    joint_msg = JointState()
    joint_msg.name = joint_names
    joint_msg.position = joint_pos
    

    return GetJointValuesResponse(joint_msg) 

def get_joint_values_server():
    rospy.init_node('get_joint_values', anonymous=True)
    s = rospy.Service('inverse_kinematics', GetJointValues, handle_get_joint_values)
    rospy.spin()


if __name__ == "__main__":
    get_joint_values_server()