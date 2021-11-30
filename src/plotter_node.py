#!/usr/bin/env python3
import numpy as np
import math as m
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

ref_val = 0
time = 0
start = 0
startFlag = False
def get_ref_val(val):
    global ref_val, startFlag, start, time
    val = val.data
    if(val != ref_val):
        startFlag = True
        start = time
        print("started rec ", ref_val, val)
    ref_val = val
    
def get_time(clock):
    global time
    time = clock.clock.secs*(10**9)+clock.clock.nsecs 
    # print(time)
    # print(clock.clock, clock)

listTime = []
listPosition = []
listRefVal = []
def forward_kinematics(pt):
    global startFlag, start, time
    pos = (float)(pt.position[0])
    if(startFlag):
        listTime.append((time - start)/10**9)
        listPosition.append(pos)
        listRefVal.append(ref_val)
    # print(ref_val - pos)
    if((len(listTime) >= 500) and startFlag):
        print(len(listTime))
        plt.plot(listTime, listPosition)
        plt.plot(listTime, listRefVal)
        plt.show()
        startFlag = False
        listPosition.clear()
        listRefVal.clear()
        listTime.clear()
    rospy.sleep(0.01)
    # pt.position[0]



if __name__=='__main__':
    try:
        rospy.init_node('Suscriber')
        start = rospy.Time.now()
        rospy.Subscriber('/robot/joint_states', JointState, forward_kinematics)
        rospy.Subscriber('/robot/joint3_position_controller/command', Float64, get_ref_val)
        rospy.Subscriber('/clock', Clock, get_time)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # forward_kinematics(0.9, 1.57, 1.57)