#!/usr/bin/env python
# license removed for brevity

'''
This is the implementation of a simple ROS-node which publishes Joint_Angle messages. 

Purpose: testing
'''

import rospy
from std_msgs.msg import String
from spinn_ros_msgs.msg import Myo_Joint_Angle


def talker():
    
    pub = rospy.Publisher('/roboy/middleware/JointAngle', Myo_Joint_Angle, queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(0.2) 
    while not rospy.is_shutdown():
        #rospy.loginfo(hello_str)
        joint_angle = Myo_Joint_Angle()
	joint_angle.degree = 90
	pub.publish(joint_angle)
	
        rate.sleep()

	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
