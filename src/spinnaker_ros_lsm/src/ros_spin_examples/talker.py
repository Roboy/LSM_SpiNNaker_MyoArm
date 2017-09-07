#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub2 = rospy.Publisher('test_image',Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.2) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
	test_image = Image()
	test_image.header.stamp = rospy.Time.now()
	test_image.height = 3
	test_image.width = 3
	test_image.data = [0,255,0,0,255,0,0,255,0]
	pub2.publish(test_image)
	
        rate.sleep()

	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
