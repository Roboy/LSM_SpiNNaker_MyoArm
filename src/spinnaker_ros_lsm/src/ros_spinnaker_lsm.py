#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@author Nicolas Berberich
@date 	10.09.2017


Credits to Stephan Reith for developing the ROS-SpiNNaker interface
"""
#
import spynnaker.pyNN as pynn

from pyNN.random import NumpyRNG, RandomDistribution
from pyNN.utility import Timer

from ros_spinnaker_interface import ROS_Spinnaker_Interface
# import transfer_functions as tf
from ros_spinnaker_interface import SpikeSourcePoisson
from ros_spinnaker_interface import SpikeSinkSmoothing, SpikeSinkMultipleReadoutsConvolution

import numpy as np

timer = Timer()
# === Define parameters ===
threads = 1
rngseed = 98766987
parallel_safe = True


ts = 0.1				 # simulation timestep in ms
simulation_time = 2000  # ms

n_input_neurons   = 4
n_readout_neurons = 2	 #
n_reservoir_neurons = 59
exc_rate = 0.8 # 80% of reservoir neurons are excitatory

n_reservoir_exc = int(np.ceil(n_reservoir_neurons*exc_rate))
n_reservoir_inh = n_reservoir_neurons - n_reservoir_exc



pynn.setup(timestep=ts, min_delay=ts, max_delay=2.0*ts)

pynn.set_number_of_neurons_per_core('IF_curr_exp', 100)      # this will set 100 neurons per core


#======================================================
#===
#===        Define Neural Populations
#===
#======================================================

############# Reservoir #################

# set up the reservoir population
celltype=pynn.IZK_cond_exp()
cell_params = {}
exc_cells = Population(n_reservoir_exc, pynn.IF_curr_exp, cell_params, label="Excitatory_Cells")
inh_cells = Population(n_reservoir_inh, celltype, cell_params, label="Inhibitory_Cells")

# set up the readout population
readout_neurons = pynn.Population(size=n_readout_neurons, cellclass=pynn.IF_curr_exp, cellparams={}, label='readout_neurons')


input_interface = ROS_Spinnaker_Interface(
        n_neurons_source=n_input_neurons,           
        Spike_Source_Class=SpikeSourcePoisson,      
                                                    
        ros_topic_send='to_spinnaker',              
       
        clk_rate=1000,                              
        ros_output_rate=10)                         


output_interface = ROS_Spinnaker_Interface(
        Spike_Sink_Class=SpikeSinkMultipleReadoutsConvolution,    
                                                    
        ros_topic_recv='from_spinnaker',            
                                    
        output_population=readout_neurons,
        clk_rate=1000,                              
        ros_output_rate=10)                         
                                                 



#======================================================
#===
#===        Define Neural Projections
#===
#======================================================

# Build your network, run the simulation and optionally record the spikes and voltages.
rconn=0.1
ext_conn = FixedProbabilityConnector(rconn, weights=0.1)



pynn.Projection(input_interface, reservoir, pynn.AllToAllConnector(weights=0.5, delays=1))



pynn.Projection(reservoir, readout_neurons, pynn.AllToAllConnector(weights=0.5, delays=1))





readout_neurons.record()
readout_neurons.record_v()

# run the network and measure the time it takes
timer.start()

pynn.run(simulation_time)

simCPUTime = timer.diff()



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

writeCPUTime = timer.diff()

print "Simulation time : %g s" % simCPUTime
print "Writing time : %g s" % writeCPUTime