#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from foundations_assignment.srv import GetJointValues, GetJointValuesResponse

def inverse_kinematics_client(pos):
    rospy.wait_for_service('inverse_kinematics')
    try:
        inverse_kinematics = rospy.ServiceProxy('inverse_kinematics',GetJointValues)
        resp1 = inverse_kinematics(pos)
        return resp1.joints
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)


if __name__ == "__main__":
    x_pos = float(sys.argv[1])
    y_pos = float(sys.argv[2])
    z_pos = float(sys.argv[3])
    
    pose = Pose()
    pos = Point(x_pos,y_pos,z_pos)
    
    pose.position = pos
    
    print(pose)
    result = inverse_kinematics_client(pose)
    print(result)
    rospy.loginfo(result)