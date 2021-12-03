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
pos = 0
def get_ref_val(val):
    global ref_val, startFlag, start, time, diffVal, pos, riseTimeStartFlag, riseTimeEndFlag
    val = val.data
    if(val != ref_val):
        startFlag = True
        start = time
        diffVal = val - pos
        riseTimeEndFlag = False
        riseTimeStartFlag = False
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
maxVal = 0
settledFlag = False
riseTimeStartFlag = False
riseTimeStart = 0
riseTimeEnd = 0
riseTimeEndFlag = False
diffVal = 0
def plotter(pt):
    global startFlag, start, time, maxVal, settledFlag, pos, riseTimeStartFlag, riseTimeEndFlag, riseTimeStart, riseTimeEnd

    pos = (float)(pt.position[0])
    if(startFlag):
        listTime.append((time - start)/10**9)
        listPosition.append(pos)
        maxVal = max(maxVal, pos)
        listRefVal.append(ref_val)
        if((abs(ref_val-pos) < 0.0003) and not settledFlag):
            print("Overshoot = " + str(ref_val - maxVal))
            print("Peak Time = " + str(listTime[listPosition.index(maxVal)]/10**6) + "ms")
            print("Settling Time = " + str((time-start)/10**6) + "ms")
            settledFlag = True
        if(not riseTimeStartFlag and not riseTimeEndFlag and (pos > 0.1*diffVal) and (pos < 0.9*diffVal)):
            riseTimeStartFlag = True
            riseTimeStart = (time-start)/10**6
        if(riseTimeStartFlag and not riseTimeEndFlag and (pos > 0.9*diffVal)):
            riseTimeEndFlag = True
            riseTimeEnd = (time-start)/10**6
            print("Rise Time = " + str(riseTimeEnd - riseTimeStart) + "ms")


    # print(ref_val - pos)
    if((len(listTime) >= 500) and startFlag):
        print("Steady State Error = " + str(listRefVal[len(listRefVal) - 1] - listPosition[len(listPosition) - 1]))
        plt.plot(listTime, listPosition)
        plt.plot(listTime, listRefVal)
        plt.show()
        startFlag = False
        settledFlag = False
        listPosition.clear()
        listRefVal.clear()
        listTime.clear()
    rospy.sleep(0.01)
    # pt.position[0]



if __name__=='__main__':
    try:
        rospy.init_node('Suscriber')
        start = rospy.Time.now()
        rospy.Subscriber('/robot/joint_states', JointState, plotter)
        rospy.Subscriber('/robot/joint3_position_controller/command', Float64, get_ref_val)
        rospy.Subscriber('/clock', Clock, get_time)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # forward_kinematics(0.9, 1.57, 1.57)