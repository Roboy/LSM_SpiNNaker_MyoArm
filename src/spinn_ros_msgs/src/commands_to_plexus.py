#!/usr/bin/env python

'''
@author	Nicolas Berberich
@date	07.09.2017

ROS node for converting Myo_Two_Motors messages to Roboy MotorCommand messages.
'''


import rospy
from std_msgs.msg import String
from spinn_ros_msgs.msg import Myo_Two_Motors
from roboy_communication_middleware.msg import MotorCommand
import numpy as np

def convert_angle():

    '''
    #topic: /roboy/middleware/MotorCommand
    #msg: roboy_communication_middleware/MotorCommand
    uint8[] motors
    int32[] setPoints
    '''    

    '''
    #topic: /roboy/middleware/JointAngle
    #msg: spinn_ros_msgs/Joint_Angle
    int32 encoder
    int32 degree
    float32 radian
    '''

    rospy.init_node('convert_joint_angle', anonymous=True)

    rospy.Subscriber("/roboy/middleware/JointStatus", JointStatus, joint_callback)
    
    rate = rospy.Rate(10) # 10hz
    
    rospy.spin()


def joint_callback(data):
    '''
    Converts the received encoder value from the joint sensor into degrees and radians and publishes it as a Joint_Angle message
    '''

    joint_angle = Myo_Joint_Angle()
    encoder_value = data.absAngles[0]
    joint_angle.encoder = encoder_value
    joint_angle.degree = int(float(encoder_value)/4096*360-230)     # decode to degree and center around 0
    joint_angle.radian = float(joint_angle.degree)/360*2*np.pi        # decode to radian and center around 0
        
    pub = rospy.Publisher('/roboy/middleware/JointAngle', Myo_Joint_Angle, queue_size=10)
    pub.publish(joint_angle)

if __name__ == '__main__':
    try:
        convert_angle()
    except rospy.ROSInterruptException:
        pass
