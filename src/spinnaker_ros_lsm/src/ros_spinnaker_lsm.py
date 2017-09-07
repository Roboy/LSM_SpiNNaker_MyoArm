#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@author Nicolas Berberich
@date 	02.09.2017

This script takes the ROS-messages from the rostopic /to_spinnaker and sends them to the input neuron
population. This input population is then connected to the reservoir of the Liquid State Machine which 
itself is connected to the two readout neurons. The spikes occuring in the readout neurons are then send 
over the neural decoder (neural transfer function) to the rostopic /from_spinnaker. 
The message type send is std_msgs/Int64.

Credits to Stephan Reith for developing the ROS-SpiNNaker interface
"""

import spynnaker.pyNN as pynn

from ros_spinnaker_interface import ROS_Spinnaker_Interface
# import transfer_functions as tf
from ros_spinnaker_interface import SpikeSourcePoisson
from ros_spinnaker_interface import SpikeSinkSmoothing


ts = 0.1				 # simulation timestep in ms
simulation_time = 10000  # ms

n_input_neurons   = 1
n_readout_neurons = 2	 # 




pynn.setup(timestep=ts, min_delay=ts, max_delay=2.0*ts)


# set up the readout population
readout_neurons = pynn.Population(size=n_readout_neurons, cellclass=pynn.IF_curr_exp, cellparams={}, label='readout_neurons')


# The ROS_Spinnaker_Interface just needs to be initialised. The following parameters are possible:
ros_interface = ROS_Spinnaker_Interface(
        n_neurons_source=n_input_neurons,                 # number of neurons of the injector population
        Spike_Source_Class=SpikeSourcePoisson,   # the transfer function ROS Input -> Spikes you want to use.
        Spike_Sink_Class=SpikeSinkSmoothing,     # the transfer function Spikes -> ROS Output you want to use.
                                                    # You can choose from the transfer_functions module
                                                    # or write one yourself.
        output_population=readout_neurons,                      # the pynn population you wish to receive the
                                                    # live spikes from.
        ros_topic_send='to_spinnaker',              # the ROS topic used for the incoming ROS values.
        ros_topic_recv='from_spinnaker',            # the ROS topic used for the outgoing ROS values.
        clk_rate=1000,                              # mainloop clock (update) rate in Hz.
        ros_output_rate=10)                         # number of ROS messages send out per second.


# Build your network, run the simulation and optionally record the spikes and voltages.
pynn.Projection(ros_interface, readout_neurons, pynn.OneToOneConnector(weights=5, delays=1))


readout_neurons.record()
readout_neurons.record_v()

pynn.run(simulation_time)

spikes = readout_neurons.getSpikes()

pynn.end()

# Plot
import pylab

spike_times = [spike[1] for spike in spikes]
spike_ids = [spike[0] for spike in spikes]

pylab.plot(spike_times, spike_ids, ".")
pylab.xlabel('Time (ms)')
pylab.ylabel('Neuron ID')
pylab.title('Spike Plot')
pylab.xlim(xmin=0)
pylab.show()
