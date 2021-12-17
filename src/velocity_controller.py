#!/usr/bin/env python3
import numpy as np
import math as m
from foundations_assignment.srv import GetJointVelocities
import rospy
from rospy.rostime import switch_to_wallclock
from rospy.service import ServiceException
from std_msgs.msg import Float64
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist

q1Pos = -1.27
q2Pos = -0.3
q3Pos = 0.0

# q1Pos = 1.57
# q2Pos = -2.8973
# q3Pos = 0.0

p_q1 = 10.0
p_q2 = 10.0
p_q3 = 10.0

d_q1 = 1.0
d_q2 = 1.0
d_q3 = 1.0

flag = False
refFlag = False
joint1_velocity_publisher = []
joint2_velocity_publisher = []
joint3_velocity_publisher = []
refPose = Pose()
curPose = Pose()
def control(msg):
    global q1Cur, q2Cur, q3Cur
    q1Cur = msg.velocity[0]
    q2Cur = msg.velocity[1]
    q3Cur = msg.velocity[2]   
# def getEndEffectorPose(msg):
#     global refPose, curPose, refFlag
#     curPose = msg
#     if refFlag:
#         refPose = curPose
#         refFlag = False

if __name__=='__main__':
    try:
        rospy.init_node('velocity_controller_node')
        rate = rospy.Rate(100)
        joint1_position_publisher = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        joint2_position_publisher = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        joint3_position_publisher = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)

        joint1_velocity_publisher = rospy.Publisher('/robot/joint1_velocity_controller/command', Float64, queue_size=1)
        joint2_velocity_publisher = rospy.Publisher('/robot/joint2_velocity_controller/command', Float64, queue_size=1)
        joint3_velocity_publisher = rospy.Publisher('/robot/joint3_velocity_controller/command', Float64, queue_size=1)


        rospy.Subscriber('/robot/joint_states', JointState, control)
        # rospy.Subscriber('/robot/pose', Pose, getEndEffectorPose)
        rospy.sleep(1)

        # move away from singularity
        joint1_position_publisher.publish(q1Pos)
        joint2_position_publisher.publish(q2Pos)
        joint3_position_publisher.publish(q3Pos)
        rospy.sleep(5)
        # refFlag = True

        rospy.wait_for_service('/robot/controller_manager/switch_controller')
        try:
            switch_service = rospy.ServiceProxy('/robot/controller_manager/switch_controller', SwitchController)
            start_controllers = ['joint1_velocity_controller', 'joint2_velocity_controller', 'joint3_velocity_controller']
            stop_controllers = ['joint1_position_controller', 'joint2_position_controller', 'joint3_position_controller']
            strictness = 2
            start_asap = False
            timeout = 0.0
            res = switch_service(start_controllers, stop_controllers, strictness, start_asap, timeout)
            print("Switched Controllers")
        except rospy.ServiceException as e:
            print("Switch Service Call Failed")
            print(e.what())
        flag = True
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 1
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        rospy.wait_for_service('/robot/get_joint_velocities_service')
        try:
            while True:
                f = open("src/foundations_assignment/velocity_plot.csv", "a")
                get_joint_velocities_service = rospy.ServiceProxy('/robot/get_joint_velocities_service', GetJointVelocities)
                print("Twist: ", twist)
                q = get_joint_velocities_service(twist)
                print("Q = ", q)
                joint1_velocity_publisher.publish(q.q[0].data)
                joint2_velocity_publisher.publish(q.q[1].data)
                joint3_velocity_publisher.publish(q.q[2].data)
                val = ""+str(q.q[0].data)+","+str(q.q[1].data)+","+str(q.q[2].data)+","+str(q1Cur)+","+str(q2Cur)+","+str(q3Cur)+"\n"
                f.write(val)
                f.close()
                rospy.sleep(0.1)
        except rospy.ServiceException as e:
            print("Service Call Failed")
            print(e.what())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # forward_kinematics(0.9, 1.57, 1.57)