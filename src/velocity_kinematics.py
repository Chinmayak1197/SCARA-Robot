#!/usr/bin/env python3
import numpy as np
import math as m
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_matrix
from foundations_assignment.srv import GetJointVelocities, GetJointVelocitiesResponse, GetTwist, GetTwistResponse
L1 = 0.1
L2 = 1
L3 = 1 - 0.075
L4 = 1

J =[]


def tdh(theta, d, a, alpha):
    c = m.cos(theta)
    s = m.sin(theta)
    t = m.cos(alpha)
    y = m.sin(alpha)
    t1 =[[c,-s*t,s*y,a*c],[s,c*t,-c*y,a*s],[0,y,t,d],[0,0,0,1]]
    return t1

def get_twist(GetTwist):
    global J
    q0 = GetTwist.q[0].data
    q1 = GetTwist.q[1].data
    q2 = GetTwist.q[2].data
    Q = np.array([[q0], [q1], [q2]])
    # print(np.shape(Q))
    # print(T1,T2,T3,T4,T)
    
    # print(J, np.shape(J))
    # E = J*Q
    # print("E: ", E, np.shape(E))
    J = np.transpose(J)
    # print(J, np.shape(J), np.shape(Q))
    E = np.matmul(J,Q)
    # print(E, np.shape(E))
    twist = Twist()
    twist.linear.x  = E[0][0]
    twist.linear.y  = E[1][0]
    twist.linear.z  = E[2][0]
    twist.angular.x = E[3][0]
    twist.angular.y = E[4][0]
    twist.angular.z = E[5][0]
    # print(twist)
    return GetTwistResponse(twist)

def get_joint_velocities(GetJointVelocities):
    global J
    twist = GetJointVelocities.twist
    # print(twist)
    E = np.array([[twist.linear.x], [twist.linear.y], [twist.linear.z], [twist.angular.x], [twist.angular.y], [twist.angular.z]])
    # print(E)
    inverse_J = np.linalg.pinv(J)
    # print(inverse_J)
    # print(np.shape(inverse_J), np.shape(E))
    inverse_J = np.transpose(inverse_J)
    Q = np.matmul(inverse_J, E)
    # print(Q)
    q0 = Float64()
    q0.data = Q[0][0]
    q1 = Float64()
    q1.data = Q[1][0]
    q2 = Float64()
    q2.data = Q[2][0]
    q = [q0, q1, q2]
    # print(q)

    return GetJointVelocitiesResponse(q)

    
    

def forward_kinematics(pt):
    global J
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

    H10 = np.array(T1)
    H20 = np.dot(T1,T2)
    H30 = np.dot(H20,T3)
    H40 = np.array(T)
    O00 = np.array([0, 0, 0])
    O10 = H10[0:3,3]
    O20 = H20[0:3,3]
    O30 = H30[0:3,3]
    O40 = H40[0:3,3]
    Z00 = np.array([0, 0, 1])
    Z10 = H10[0:3,2]
    Z20 = H20[0:3,2]
    Z30 = H30[0:3,2]
    Z40 = H40[0:3,2]

    JV1 = np.cross(Z10, (O40 - O10))
    JV2 = np.cross(Z20, (O40 - O20))
    JV3 = Z30
    JW1 = Z10
    JW2 = Z20
    JW3 = np.array([0, 0, 0])
    J1 = np.concatenate((JV1, JW1))
    J2 = np.concatenate((JV2, JW2))
    J3 = np.concatenate((JV3, JW3))
    J = np.array([J1,J2,J3])
    # print(J)
    pose = Pose()

    quat = quaternion_from_matrix(T)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    pose.position.x = T[0][3]
    pose.position.y = T[1][3]
    pose.position.z = T[2][3]
    pub.publish(pose)

    # print(pose)
   



if __name__=='__main__':
    try:
        rospy.init_node('to_twist_node')
        rospy.Subscriber('/robot/joint_states', JointState, forward_kinematics)
        pub = rospy.Publisher('/robot/pose', Pose, queue_size=10)
        twistService = rospy.Service('/robot/get_twist_service', GetTwist, get_twist)
        jointVelocitiesService = rospy.Service('/robot/get_joint_velocities_service', GetJointVelocities, get_joint_velocities)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # forward_kinematics(0.9, 1.57, 1.57)