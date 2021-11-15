#!/usr/bin/env
import numpy as np
import math as m
import rospy
from sensor_msgs.msg import JointState
L1 = 0.1
L2 = 1
L3 = 1 - 0.075
L4 = 1


def tdh(theta, d, a, alpha):
    c = m.cos(theta)
    s = m.sin(theta)
    t = m.cos(alpha)
    y = m.sin(alpha)
    t1 =[[c,-s*t,s*y,a*c],[s,c*t,-c*y,a*s],[0,y,t,d],[0,0,0,1]]
    return t1


def forward_kinematics(pt):

    q1 = pt.position[0]
    q2 = pt.position[1]
    q3 = pt.position[2]
    L5 = q3
    T1=tdh(0,  L1,  0, 0)
    T2=tdh(q1,  L2,  L3, 0)
    T3=tdh(q2,   0, L4,  m.pi)
    T4=tdh(0,   L5, 0,  0)

    TQ =np.dot(T3,T4)
    TQ1 =np.dot(T2,TQ)
    T = np.dot(T1,TQ1)
    print(T)
   



if __name__=='__main__':
    try:
        rospy.init_node('Suscriber')
        rospy.Subscriber('/robot/joint_states', JointState, forward_kinematics)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # forward_kinematics(0.9, 1.57, 1.57)