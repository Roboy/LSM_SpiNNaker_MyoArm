#!/usr/bin/env python


'''
This ROS-node runs the LSM in NEST. 

It receives the arm's joint angle and outputs the readout neurons' spike rates. 

Subscribes to: 
		- Topic: /joint_angle           Message-type: Myo_Joint_Angle

Publishes to: 
		- Topic: /alpha_readout_rates   Message-type: Pop_List
		- Topic: /mean_readout_rates    Message-type: Pop_List


@author: Nicolas Berberich


'''


import pyNN.nest as p
from pyNN.random import NumpyRNG, RandomDistribution
from pyNN.utility import Timer
import matplotlib.pyplot as plt
import pylab
import numpy as np
from scipy import signal


import rospy
from std_msgs.msg import String
from spinn_ros_msgs.msg import Myo_Joint_Angle
from spinn_ros_msgs.msg import Pop_List


def network():
    rospy.init_node('lsm_node')
    rate = rospy.Rate(10) # 10hz
    
    rospy.Subscriber('joint_angle', Myo_Joint_Angle, callback)

    rospy.loginfo('starting---------------')
    rospy.spin()
    #while True:
    #    rospy.loginfo_throttle(10, "This message will print every 10 seconds")

def mean_decoding(spikes, dt):

    n_neurons= int(spikes[:,0].max())+1

    spiketrains = []
    # for all readout neurons
    for i in range(n_neurons): 
        spiketrain = spikes[np.where(spikes[:,0]==i),1]
        spiketrain=np.squeeze(spiketrain)
        spiketrains.append(spiketrain)
     # Spiketrains is a list with the spiketime-arrays for each neuron as elements
    mean_rates = np.zeros((n_neurons,))
    
    for i, spiketrain in enumerate(spiketrains):
        mean_rates[i]=len(spiketrain)
    return mean_rates
    


def alpha_decoding(spikes,dt):
    # compute instantaneous spike rate at last spike

    f = lambda x: x*0.3*np.exp(2-x*0.3)
    n_neurons= int(spikes[:,0].max())+1

    spiketrains = []
    # for all readout neurons
    for i in range(n_neurons): 
        spiketrain = spikes[np.where(spikes[:,0]==i),1]
        spiketrain=np.squeeze(spiketrain)
        spiketrains.append(spiketrain)
     # Spiketrains is a list with the spiketime-arrays for each neuron as elements

    last_spike = np.max(spiketrains)
    alpha_rates=np.zeros((n_neurons,))

    for i, spiketrain in enumerate(spiketrains):
	for spike in spiketrain:
            # weight spike based on time
	    alpha_rates[i]+=f(last_spike-spike)
	# TODO: add scaling factor
	#scaling_factor = 1/np.sum(alpha_kernel)*1/dt
    
    return alpha_rates

def callback(data_input):

    #====================================================================
    # Unpacking the Joint Angle Message
    #====================================================================
    global message
    message = data_input.degree
    rospy.loginfo('=====> received joint angle in degree %r', message)
    print message

    if type(message) != int:
    	input_rates = list(message)
	n_input_neurons = len(input_rates)  
    else:
	input_rates = message
	n_input_neurons = 1
	

    #msg_list= [int(msg.encode('hex'),16) for msg in message]
    

    timer = Timer()
    dt = 0.1
    p.setup(timestep=dt) # 0.1ms


    #====================================================================
    # Defining the LSM
    #====================================================================

    n_res=2000
    w_exc_b=0.2
    w_inh_b=-0.8
    rout_w_exc=20
    rout_w_inh=-80

    n_readout_neurons   = 2
    n_reservoir_neurons = n_res
    n_res = n_reservoir_neurons
    exc_rate            = 0.8 # % of excitatory neurons in reservoir

    n_exc = int(round(n_reservoir_neurons*exc_rate))
    n_inh = n_reservoir_neurons-n_exc
    izh_celltype = p.native_cell_type('izhikevich')
    if_celltype = p.IF_curr_exp
    celltype = if_celltype
    
    spike_source = p.native_cell_type('poisson_generator')
    inp_pop=p.Population(n_input_neurons*10,spike_source,{'rate':input_rates})
    
    exc_cells = p.Population(n_exc, celltype, label="Excitatory_Cells")
    inh_cells = p.Population(n_inh, celltype, label="Inhibitory_Cells")

    # initialize with a uniform random distributin
    # use seeding for reproducability
    rngseed = 98766987
    parallel_safe = True
    rng = NumpyRNG(seed=rngseed, parallel_safe=parallel_safe)

    unifDistr = RandomDistribution('uniform', (-70,-65), rng=rng)
    inh_cells.initialize('V_m',unifDistr)
    exc_cells.initialize('V_m',unifDistr)
    
    readout_neurons = p.Population(2, celltype, label="readout_neuron")
    
    inp_weight=3.
    inp_delay =1

    inp_weight_distr = RandomDistribution('normal', [inp_weight, 1e-3], rng=rng)

    # connect each input neuron to 30% of the reservoir neurons
    inp_conn = p.FixedProbabilityConnector(p_connect=0.3,weights =inp_weight_distr, delays=inp_delay)

    connections = {}
    connections['inp2e'] = p.Projection(inp_pop, exc_cells, inp_conn)
    connections['inp2i'] = p.Projection(inp_pop, inh_cells, inp_conn)

    pconn = 0.01      # sparse connection probability

    # scale the weights w.r.t. the network to keep it stable
    w_exc = w_exc_b/np.sqrt(n_res)      # nA
    w_inh = w_inh_b/np.sqrt(n_res)      # nA
    
    delay_exc = 1      # defines how long (ms) the synapse takes for transmission
    delay_inh = 1

    weight_distr_exc = RandomDistribution('normal', [w_exc, 1/n_res], rng=rng)
    weight_distr_inh = RandomDistribution('normal', [w_inh, 1/n_res], rng=rng)
    exc_conn = p.FixedProbabilityConnector(pconn, weights=weight_distr_exc, delays=delay_exc)
    inh_conn = p.FixedProbabilityConnector(pconn, weights=weight_distr_inh, delays=delay_inh)

    connections['e2e'] = p.Projection(exc_cells, exc_cells, exc_conn, target='excitatory')
    connections['e2i'] = p.Projection(exc_cells, inh_cells, exc_conn, target='excitatory')
    connections['i2e'] = p.Projection(inh_cells, exc_cells, inh_conn, target='inhibitory')
    connections['i2i'] = p.Projection(inh_cells, inh_cells, inh_conn, target='inhibitory')
    
    
    
    rout_conn_exc = p.AllToAllConnector(weights=rout_w_exc, delays=delay_exc)
    rout_conn_inh = p.AllToAllConnector(weights=rout_w_inh, delays=delay_exc)

    
    

    connections['e2rout'] = p.Projection(exc_cells, readout_neurons, rout_conn_exc, target='excitatory')
    connections['i2rout'] = p.Projection(inh_cells, readout_neurons, rout_conn_inh, target='inhibitory')
    
    readout_neurons.record()
    exc_cells.record()
    inh_cells.record()
    inp_pop.record()
    
    
    p.run(20)

    r_spikes = readout_neurons.getSpikes()
    exc_spikes = exc_cells.getSpikes()
    inh_spikes = inh_cells.getSpikes()
    inp_spikes = inp_pop.getSpikes()

    rospy.loginfo('=====> shape of r_spikes %r', np.shape(r_spikes))

    #====================================================================
    # Compute Readout Spike Rates
    #====================================================================
    
  
    alpha_rates = alpha_decoding(r_spikes,dt)
    mean_rates  = mean_decoding(r_spikes,dt)

    #====================================================================
    # Publish Readout Rates
    #====================================================================

    # TODO: error handling if r_spikes is empty
    pub = rospy.Publisher('/alpha_readout_rates', Pop_List, queue_size=10)
    alpha_readout_rates = Pop_List
    alpha_readout_rates = alpha_rates
    pub.publish(alpha_readout_rates)

    pub = rospy.Publisher('/mean_readout_rates', Pop_List, queue_size=10)
    mean_readout_rates = Pop_List
    mean_readout_rates = mean_rates
    pub.publish(mean_readout_rates)
    
    

if __name__ == '__main__':
    try:
        network()
    except rospy.ROSInterruptException:
        pass
