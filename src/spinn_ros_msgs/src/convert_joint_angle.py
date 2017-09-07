#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from spinn_ros_msgs.msg import Joint_Angle
from roboy_communication_middleware.msg import JointStatus
import numpy as np

def convert_angle():

    '''
    #topic: /roboy/middleware/JointStatus
    #msg: roboy_communication_middleware/JointStatus
    int32 id
    uint32[] absAngles
    uint32[] relAngles
    uint32[] tacho
    uint8[] agcGain
    bool[] tooFar
    bool[] tooClose
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

    joint_angle = Joint_Angle()
    encoder_value = data.absAngles[0]
    joint_angle.encoder = encoder_value
    joint_angle.degree = int(float(encoder_value)/4096*360-180)     # decode to degree and center around 0
    joint_angle.radian = float(encoder_value)/4096*np.pi - np.pi/2  # decode to radian and center around 0
        
    pub = rospy.Publisher('/roboy/middleware/JointAngle', Joint_Angle, queue_size=10)
    pub.publish(joint_angle)

if __name__ == '__main__':
    try:
        convert_angle()
    except rospy.ROSInterruptException:
        pass