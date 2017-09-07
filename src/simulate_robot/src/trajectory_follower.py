#!/usr/bin/env python


import pyNN.nest as p
from pyNN.random import NumpyRNG, RandomDistribution
from pyNN.utility import Timer
import matplotlib.pyplot as plt
import pylab
import numpy as np
from scipy import signal


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def follower():
    rospy.init_node('simple_network_node')
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, test_callback)
    
    # define the trajectory which should be followed
    x = np.linspace(0, 20, 100)
    traj=np.sin(x)

    rospy.loginfo('starting---------------')
    rospy.spin()
    #while True:
    #    rospy.loginfo_throttle(10, "This message will print every 10 seconds")


def test_callback(data_input):
    global message
    message = data_input.actual.positions
    msg_list = list(message)

    #msg_list[0] = int(message[0].encode('hex'),16)
    #for i in
    #msg_list = int(message.encode('hex'),16)

    #print('============= Received image data.',message)
    rospy.loginfo('=====received data %r', msg_list[0])
    timer = Timer()
    dt = 0.1
    p.setup(timestep=dt) # 0.1ms


    pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    command = FollowJointTrajectoryActionGoal()
    command.header.stamp = rospy.Time.now()
    command.goal.trajectory.joint_names = ['elbow']
    point = JointTrajectoryPoint()
    point.positions = [rate_command/10]
    point.time_from_start = rospy.Duration(1)
    command.goal.trajectory.points.append(point)
    pub.publish(command)
    rospy.loginfo('=====send command %r', command.goal.trajectory.points[0])



    print("now plotting the network---------------")
    rospy.loginfo('--------now plotting---------------')
    n_panels = sum(a.shape[1] for a in pop_1_data.segments[0].analogsignalarrays) + 2
    plt.subplot(n_panels, 1, 1)
    plot_spiketrains(pop_1_data.segments[0])
    panel = 3
    for array in pop_1_data.segments[0].analogsignalarrays:
        for i in range(array.shape[1]):
            plt.subplot(n_panels, 1, panel)
            plot_signal(array, i, colour='bg'[panel%2])
            panel += 1
    plt.xlabel("time (%s)" % array.times.units._dimensionality.string)
    plt.setp(plt.gca().get_xticklabels(), visible=True)#

    #plt.show()
    #fig1.show()
    #plt.savefig("~/Spiking-Neural-Networks-on-Robotino/network_output.jpg")



if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass
