# LSM_SpiNNaker_MyoArm
Repository for running a Liquid State Machine on SpiNNaker or optionally on NEST for the control of a 1-joint myorobotic arm. The communication is based on ROS.


# Structure

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/interfaces.png "Logo Title Text 1")


# Requirements

This project uses several software packages with delicate interdependencies. The sPyNNaker interface between PyNN and the SpiNNaker machine for example only supports PyNN versions 0.7.x, but not the more recent 0.8 or 0.9 releases. For every PyNN version there is furthermore only one NEST version which is supported, newer versions typically aren't. 

In the table below, you can see the set of software packages and their respective versions that are required. 

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/requirements.png "Logo Title Text 1")


Additionally, this project uses NumPy. For running the LSM with NEST as back-end you need NumPy version 1.13.0 while the sPyNNaker interface uses NumPy version 1.12.0.

We have also included additional repositories that are being developed separately.
For the most recent status of the code and documentation, have a look at:

https://github.com/Erzcoder/ros_spinnaker_interface.git

and

https://github.com/Erzcoder/spinn_ros_msgs.git



# System Architecture

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/system_architecture.png "Logo Title Text 1")

# Connection from Robot to the LSM

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/robot_to_lsm.png "Logo Title Text 1")



# Connection from the LSM to the Robot

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/lsm_to_robot.png "Logo Title Text 1")



