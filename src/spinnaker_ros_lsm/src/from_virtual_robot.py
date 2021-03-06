#!/usr/bin/env python

'''
@author	Nicolas Berberich
@date	07.09.2017

ROS node for converting ros_control joint state values from the virtual robot to degrees and radians.
'''


import rospy
from std_msgs.msg import String
from spinn_ros_msgs.msg import Myo_Joint_Angle
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np

def convert_angle():

    '''
    #topic: /arm_controller/state
    #msg: control_msgs/JointTrajectoryControllerState
    Header header
    string[] joint_names
    trajectory_msgs/JointTrajectoryPoint desired
    trajectory_msgs/JointTrajectoryPoint actual
    trajectory_msgs/JointTrajectoryPoint error
    '''    

    '''
    #topic: /roboy/middleware/JointAngle
    #msg: spinn_ros_msgs/Joint_Angle
    int32 encoder
    int32 degree
    float32 radian
    '''

    rospy.init_node('from_virtual_robot', anonymous=True)

    rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, joint_callback)
    
    rate = rospy.Rate(10) # 10hz
    
    rospy.spin()


def joint_callback(data):
    '''
    Converts the received ros_control state value into degrees and radians and publishes it as a Joint_Angle message
    '''
    global message
    message = data_input.actual.positions
    joint_positon = list(message)

    joint_angle = Myo_Joint_Angle()
    joint_angle.encoder = encoder_value
    joint_angle.degree = joint_position     # decode to degree and center around 0
    joint_angle.radian = float(joint_angle.degree)/360*2*np.pi        # decode to radian and center around 0
        
    pub = rospy.Publisher('/roboy/middleware/JointAngle', Myo_Joint_Angle, queue_size=10)
    pub.publish(joint_angle)

if __name__ == '__main__':
    try:
        convert_angle()
    except rospy.ROSInterruptException:
        pass
