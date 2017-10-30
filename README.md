# LSM_SpiNNaker_MyoArm
Repository for running a Liquid State Machine on SpiNNaker or optionally on NEST for the control of a 1-joint myorobotic arm. The communication is based on ROS.


# Structure

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/interfaces.png "Interfaces")

Our approach allows high flexibility by allowing the user to either run the spiking neural network on her own PC with NEST or (with almost no changes to the network specification) on the dedicated SpiNNaker neuromorphic hardware in biological real-time. The user is also free to decide between the physical Myorobotic arm or a virtual model of it.  

# Demo Video

<a href="http://www.youtube.com/watch?feature=player_embedded&v=cjA-FDAnFxs
" target="_blank"><img src="http://img.youtube.com/vi/cjA-FDAnFxs/0.jpg" 
alt="demo video" width="360" height="270" border="10" /></a>

Here you can see our LSM-based closed loop system steering a virtual Myorobotic arm. 
The LSM that we used for this demo video had a reservoir of 2000 spiking neurons. Because the simulation of these is not real-time the robot arm in the simulation moves very slowly. The arrows in the simulator show the forces exerted at the tendon end points. 

For running the same demo on your PC, you have to download this repository and delete the common_utilities folder in src. The reason for this is that the virtual and the physical robot currently use different commit versions of this package (will be made generic in the future). A saved version of common_utilities can be found in the root directory in the folder 'buffer' which you will have to add again for working with the physical robot. 

Now build the ROS workspace by running the following directly in LSM-SpiNNaker_MyoArm:

`catkin_make --pkg spinn_ros_msgs nest_ros_lsm roboy_control_learning roboy_simulation roboy_controller roboy_models`

Don't forget to source it by running `source devel/setup.bash` in the repository's root directory. 

If you have installed all requirements listed in the following section, you can now start the individual ROS-nodes (each in a separate sourced command line tab): 

```
$ rosrun nest_ros_lsm from_virtual_robot.py
$ rosrun nest_ros_lsm to_virtual_robot.py
$ rosrun nest_ros_lsm nest_lsm_node.py
$ rosrun nest_ros_lsm pop_list_interface.py 
$ roslaunch roboy_control_learning single_joint_myoarm.launch 
```

We will add launch files later on for easier usage. 

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

The blocks with round edges represent ROS-nodes and the blocks with sharp edges are ROS-topics to which these nodes subscribe or publish messages. 


# Connection from the LSM to the Robot

![](https://github.com/Roboy/LSM_SpiNNaker_MyoArm/blob/master/media/lsm_to_robot.png "LSM to Robot")

The blocks with round edges represent ROS-nodes and the blocks with sharp edges are ROS-topics to which these nodes subscribe or publish messages. 

# Explanations of the Sub-Packages

All relevant code can be found in the *src* folder. The *media* folder contains several images and the *Thesis* folder contains saved versions of scripts that were referenced in my thesis. 

In *src*: 

| Package               | Explanation |
|-----------------------|-------------|
|roboy_communication| Roboy package for communication with the physical robot. |
|roboy_plexus | Roboy package for communication with the physical robot.|
|common_utilities |Roboy package communication with the physical robot. |
|roboy_rqt_control | Roboy package containing the GUI for controlling the physical robot. |
|roboy_ros_control | Roboy package for communication with the physical and virtual robot.|
|roboy_control_learning| For simulating the virtual robot.|
|spinn_ros_msgs| Adds several custom ROS-messages. | 
|experimental_code      | Contains scripts for experimenting with the spiking neural network. |
|nest_ros_lsm| ROS-nodes for running the LSM with the NEST neural simulator and connect it to ROS and the virtual/physical robot. |
|ros_spinnaker_interface | Modified version for connecting ROS and SpiNNaker with multiple output neurons. |
|spinnaker_ros_lsm | ROS-nodes for running the LSM on SpiNNaker and connect it to ROS and the virtual/physical robot.|

The main packages are **nest_ros_lsm** and **spinnaker_ros_lsm**. 

