#!/usr/bin/env python
# license removed for brevity

'''
@author Nicolas Berberich
@date   07.09.2017

ROS node for converting Pop_List messages to Roboy MotorControl messages.

'''

import rospy
from std_msgs.msg import String
from spinn_ros_msgs.msg import Pop_List

from roboy_simulation.msg import MotorControl
import numpy as np


# TODO: should also subscribe to the joint angle to take care of the joint limit

def talker():
    rospy.init_node('to_virtual_robot')
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber('/readout_rates', Pop_List, callback)
    rospy.spin()

def transfer_fct(readout_rates):
    # add joint and motor limits
    motor_commands = [0.0,0.0]
   
    # TODO: this is only a preliminary version for testing the closed loop; needs to be added later
    motor_commands[0]=readout_rates[0]/70.
    if motor_commands[0]>2:
        motor_commands[0]=2.
    motor_commands[1]=2. - motor_commands[0]

    return motor_commands

def callback(data_input):

    
    motor_commands = MotorControl()
    readout_rates = data_input.neuron_pop
    
    motor_commands.roboyID = 0
    
    
    
    motor_commands.voltage = transfer_fct(readout_rates)
    pub = rospy.Publisher('/roboy/motor_control', MotorControl, queue_size=10) 
    
    print motor_commands
    pub.publish(motor_commands)

	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
