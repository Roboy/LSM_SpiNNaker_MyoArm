#!/usr/bin/env python
# license removed for brevity

'''
This is the implementation of a simple ROS-node which receives Myo_Two_Motors motor_command messages and publishes Myo_Joint_Angle messages. 

Purpose: testing; dummy for the robot
'''

import rospy
from std_msgs.msg import String
from spinn_ros_msgs.msg import Pop_List
from spinn_ros_msgs.msg import Myo_Two_Motors
from spinn_ros_msgs.msg import Myo_Joint_Angle



# TODO: should also subscribe to the joint angle to take care of the joint limit

def robot():
    rospy.init_node('robot')
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber('/motor_commands', Myo_Two_Motors, callback)
    rospy.spin()



def callback(data_input):

    joint_angle = Myo_Joint_Angle()
    motors = data_input.motor_commands
    
    joint_angle.degree=motors[0]
    
    
    pub = rospy.Publisher('/roboy/middleware/JointAngle', Myo_Joint_Angle, queue_size=10) 
    
    
    pub.publish(joint_angle)

	

if __name__ == '__main__':
    try:
        robot()
    except rospy.ROSInterruptException:
        pass
