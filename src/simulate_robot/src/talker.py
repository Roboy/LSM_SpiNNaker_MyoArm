#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import numpy as np


def talker():
    rospy.loginfo('starting the talker node')
    global counter
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    

    pub= rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    


    # we need a discretized sinusoid
    x = np.linspace(0, 6, 10)
    traj=np.sin(x)

    
    rospy.sleep(2)

    command = FollowJointTrajectoryActionGoal()
    command.header.stamp = rospy.Time.now()
    command.goal.trajectory.joint_names = ['elbow']
    point = JointTrajectoryPoint()
    point.positions=[math.pi/2]
    point.velocities=[1.5] #0.5
    #point.positions = [math.sin(counter/4)*math.pi] 
    # in which range is sinus defined?
    point.time_from_start = rospy.Duration(10)
    command.goal.trajectory.points.append(point)
    point.positions=[0]
    point.velocities=[1.5] #0.5
    #point.positions = [math.sin(counter/4)*math.pi] 
    # in which range is sinus defined?
    point.time_from_start = rospy.Duration(20)
    command.goal.trajectory.points.append(point)



    pub.publish(command)
    rospy.loginfo('=====send command %r', command.goal.trajectory.points[0])

    
    rospy.sleep(2)
    counter+=1

    rate.sleep()

	

if __name__ == '__main__':
    try:
        global counter
        counter=0
        talker()

    except rospy.ROSInterruptException:
        pass
