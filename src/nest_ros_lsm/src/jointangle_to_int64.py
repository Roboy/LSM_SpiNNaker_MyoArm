#!/usr/bin/env python

'''
@author	Nicolas Berberich
@date	07.09.2017

ROS node for converting the Joint_Angle ROS-message degree data field to a 64-bit integer that can be sent to SpiNNaker. 
'''


import rospy
from std_msgs.msg import Int64
from spinn_ros_msgs.msg import Myo_Joint_Angle
import numpy as np

def convert_angle():

    '''
    #topic: /to_spinnaker
    #msg: std_msgs/int64
    int64 data
    '''    

    '''
    #topic: /roboy/middleware/JointAngle
    #msg: spinn_ros_msgs/Joint_Angle
    int32 encoder
    int32 degree
    float32 radian
    '''

    rospy.init_node('jointangle_to_int64')

    rospy.Subscriber("/roboy/middleware/JointAngle", JointAngle, joint_callback)
    
    rate = rospy.Rate(10) # 10hz
    
    rospy.spin()


def joint_callback(data):
    '''
    Converts the received joint angle value (in degree) into an Int64 message
    '''

    int_value = Int64()
    int_value = int(data.degree)
      
    pub = rospy.Publisher('/to_spinnaker', Int64, queue_size=10)
    pub.publish(int_value)

if __name__ == '__main__':
    try:
        convert_angle()
    except rospy.ROSInterruptException:
        pass
