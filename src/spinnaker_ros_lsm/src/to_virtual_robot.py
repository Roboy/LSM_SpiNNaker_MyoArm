#!/usr/bin/env python
# license removed for brevity

'''
@author Nicolas Berberich
@date   07.09.2017

ROS node for converting Myo_Two_Motors messages to Roboy MotorCommand messages.

'''

import rospy
from std_msgs.msg import String
from spinn_ros_msgs.msg import Pop_List
from spinn_ros_msgs.msg import Myo_Two_Motors
from spinn_ros_msgs.msg import Myo_Two_Motors
from roboy_communication_middleware.msg import MotorCommand
import numpy as np


# TODO: should also subscribe to the joint angle to take care of the joint limit

def talker():
    rospy.init_node('transfer_function')
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber('/alpha_readout_rates', Pop_List, callback)
    rospy.spin()

def transfer_fct(readout_rates):
    motor_commands = []
    for rate in readout_rates:
        motor_commands.append(rate/100.)

    return motor_commands

def callback(data_input):

    motor_commands = Myo_Two_Motors()
    readout_rates = data_input.neuron_pop
    print readout_rates
    
    
    motor_commands.motor_commands = transfer_fct(readout_rates)
    pub = rospy.Publisher('/motor_commands', Myo_Two_Motors, queue_size=10) 
    
    print motor_commands
    pub.publish(motor_commands)

	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
