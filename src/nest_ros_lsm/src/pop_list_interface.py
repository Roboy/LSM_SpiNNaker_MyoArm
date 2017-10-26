#!/usr/bin/env python

'''
@author	Nicolas Berberich
@date	07.09.2017

ROS node that publishes the received Pop_List to a uniform topic
'''


import rospy
from spinn_ros_msgs.msg import Pop_List
import numpy as np

def change_topic():

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

    rospy.init_node('pop_list_interface')

    rospy.Subscriber("/alpha_readout_rates", Pop_List, callback)
    rospy.Subscriber("/mean_readout_rates", Pop_List, callback)
    rospy.Subscriber("/from_spinnaker", Pop_List, callback)
    
    rate = rospy.Rate(10) # 10hz
    
    rospy.spin()


def callback(data):
    '''
    Publishes the received Pop_List to a uniform topic
    '''

    readout_rates = Pop_List()
    readout_rates = data
      
    pub = rospy.Publisher('/readout_rates', Pop_List, queue_size=10)
    pub.publish(readout_rates)

if __name__ == '__main__':
    try:
        change_topic()
    except rospy.ROSInterruptException:
        pass
