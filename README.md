# LSM_SpiNNaker_MyoArm
Repository for running a Liquid State Machine on SpiNNaker or optionally on NEST for the control of a 1-joint myorobotic arm. The communication is based on ROS.


# Structure

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/interfaces.png "Interfaces")


# Demo Video

<a href="http://www.youtube.com/watch?feature=player_embedded&v=cjA-FDAnFxs
" target="_blank"><img src="http://img.youtube.com/vi/cjA-FDAnFxs/0.jpg" 
alt="demo video" width="360" height="270" border="10" /></a>

Here you can see our LSM-based closed loop system steering a virtual Myorobotic arm. 
The LSM that we used for this demo video had a reservoir of 2000 spiking neurons. Because the simulation of these is not real-time the robot arm in the simulation moves very slowly. 

A video showing the control of the physical Myorobotic arm will be added shortly. 


# Requirements

This project uses several software packages with delicate interdependencies. The sPyNNaker interface between PyNN and the SpiNNaker machine for example only supports PyNN versions 0.7.x, but not the more recent 0.8 or 0.9 releases. For every PyNN version there is furthermore only one NEST version which is supported, newer versions typically aren't. 

In the table below, you can see the set of software packages and their respective versions that are required. 

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/requirements.png "Requirements")


Additionally, this project uses NumPy. For running the LSM with NEST as back-end you need NumPy version 1.13.0 while the sPyNNaker interface uses NumPy version 1.12.0.

We have also included additional repositories that are being developed separately.
For the most recent status of the code and documentation, have a look at:

https://github.com/Erzcoder/ros_spinnaker_interface.git

and

https://github.com/Erzcoder/spinn_ros_msgs.git



# System Architecture

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/system_architecture.png "System Architecture")

# Connection from Robot to the LSM

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/robot_to_lsm.png "Robot to LSM")



# Connection from the LSM to the Robot

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/lsm_to_robot.png "LSM to Robot")



