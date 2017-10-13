#!/usr/bin/env python


'''


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


def network():
    rospy.init_node('lsm_node')
    rate = rospy.Rate(10) # 10hz
    
    rospy.Subscriber('joint_angle', Myo_Joint_Angle, callback)

    rospy.loginfo('starting---------------')
    rospy.spin()
    #while True:
    #    rospy.loginfo_throttle(10, "This message will print every 10 seconds")

def gaussian_convolution(spikes,dt):
    #----------- works only after the simulation has run; not online!!!!!!!!
    kernel_size = 10
    gaussian_kernel = signal.gaussian(kernel_size, std=2)
    scaling_factor = 1/np.sum(gaussian_kernel)*1/dt
    gauss_rate = np.convolve(spikes,gaussian_kernel,mode='same')*scaling_factor
    mean_rate = np.mean(gauss_rate)
    return mean_rate

def alpha_decoding(spikes,dt):
    #----------- works only after the simulation has run; not online!!!!!!!!
    f = lambda x: x*0.3*np.exp(2-x*0.3)
    alpha_kernel = np.zeros((kernel_size,))
    for i in range(kernel_size):
        alpha_kernel[i]=f(i)
    scaling_factor2 = 1/np.sum(alpha_kernel)*1/dt
    alpha_rate = np.convolve(spikes,alpha_kernel,mode='same')*scaling_factor
    return alpha_rate

def callback(data_input):

    #====================================================================
    # Unpacking the Joint Angle Message
    #====================================================================
    global message
    message = data_input.degree
    rospy.loginfo('=====> received joint angle in degree %r', message)

    if type(message) != int:
    	msg_list = list(message)
	n_input_neurons = len(msg_list)  
    else:
	msg_list = message
	n_input_neurons = 1
	

    #msg_list= [int(msg.encode('hex'),16) for msg in message]
    

    timer = Timer()
    dt = 0.1
    p.setup(timestep=dt) # 0.1ms


    
    rospy.loginfo('====length of input image %r', n_input_neurons)


    #====================================================================
    # Defining the LSM
    #====================================================================

    input_neuron = p.Population(n_input_neurons, p.SpikeSourcePoisson, {'rate':msg_list})

    n = 200          # number of cells
    exc_ratio = 0.8   # ratio of excitatory neurons
    n_exc = int(round(n*0.8))
    n_inh = n-n_exc
    celltype = p.Izhikevich()
    exc_cells = p.Population(n_exc, celltype, label="Excitatory_Cells")
    inh_cells = p.Population(n_inh, celltype, label="Inhibitory_Cells")

    # initialize with a uniform random distributin
    # use seeding for reproducability
    rngseed = 98766987
    parallel_safe = True

    rng = NumpyRNG(seed=rngseed, parallel_safe=parallel_safe)

    unifDistr = RandomDistribution('uniform', (-75,-65), rng=rng)
    exc_cells.initialize(v=unifDistr)
    inh_cells.initialize(v=unifDistr)

    readout_neurons = p.Population(1, celltype, label="readout_neuron")


    w_exc = 20.   # parameter than can be changed
    w_inh = 80.   # parameter than can be changed
    delay_inp = 1
    delay_exc = 1      # defines how long (ms) the synapse takes for transmission
    delay_inh = 1


    weight_distr_inp = RandomDistribution('uniform',(1,10),rng=rng)
    weight_distr_exc = RandomDistribution('normal', [w_exc, 1e-3], rng=rng)
    weight_distr_inh = RandomDistribution('normal', [w_inh, 1e-3], rng=rng)


    stat_syn_inp = p.StaticSynapse(weight =weight_distr_inp, delay=delay_inp)
    stat_syn_exc = p.StaticSynapse(weight =weight_distr_exc, delay=delay_exc)
    stat_syn_inh = p.StaticSynapse(weight =weight_distr_inh, delay=delay_inh)



    pconn = 0.01      # sparse connection probability within the reservoir
    input_conn = 0.3  # sparse connections from input to reservoir
    
    exc_conn = p.FixedProbabilityConnector(pconn, rng=rng)
    inh_conn = p.FixedProbabilityConnector(pconn, rng=rng)
    inp_conn = p.FixedProbabilityConnector(input_conn, rng=rng)
    rout_conn = p.AllToAllConnector()
    
    connections = {}
    connections['e2e'] = p.Projection(exc_cells, exc_cells, exc_conn,
                                synapse_type=stat_syn_exc, receptor_type='excitatory')
    connections['e2i'] = p.Projection(exc_cells, inh_cells, exc_conn,
                                synapse_type=stat_syn_exc,receptor_type='excitatory')
    connections['i2e'] = p.Projection(inh_cells, exc_cells, inh_conn,
                                synapse_type=stat_syn_inh,receptor_type='inhibitory')
    connections['i2i'] = p.Projection(inh_cells, inh_cells, inh_conn,
                                synapse_type=stat_syn_inh,receptor_type='inhibitory')


    connections['inp2e'] = p.Projection(input_neuron, exc_cells, inp_conn,
                                      synapse_type=stat_syn_inp,receptor_type='excitatory')
    connections['inp2i'] = p.Projection(input_neuron, inh_cells, inp_conn,
                                      synapse_type=stat_syn_inp,receptor_type='excitatory')

    connections['e2rout'] = p.Projection(exc_cells, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_exc,receptor_type='excitatory')
    connections['i2rout'] = p.Projection(inh_cells, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_inh,receptor_type='inhibitory')


    #=====================================================================
    # recording and running the network

    readout_neurons.record(['v','spikes'])
    p.run(20)
    readout_data= readout_neurons.get_data()

    spikes = readout_data.segments[0].spiketrains[0]
    mean_rate = int(gaussian_convolution(spikes,dt))
    rospy.loginfo('=====mean_rate %r', mean_rate) # mean_rate = 64
    rate_command = mean_rate
    # rate coding of the spike train

    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    #======================================================================
    # construct the output command
    # we handcrafted the transfer function to be within the joint limits of the robot

    command = Twist()
    command.linear.x = np.abs((int(mean_rate)%10-4))*10+0.24 #0.24
    command.angular.z = -(int(mean_rate)%10-4.5)*10
	#int(mean_rate)%100/100.*np.pi-np.pi/2
    pub.publish(command)

    rospy.loginfo('=====send command %r', command.angular.y)


    fig_settings = {
        'lines.linewidth': 0.5,
        'axes.linewidth': 0.5,
        'axes.labelsize': 'small',
        'legend.fontsize': 'small',
        'font.size': 8
    }
    plt.rcParams.update(fig_settings)
    fig1=plt.figure(1, figsize=(6,8))

    def plot_spiketrains(segment):
        for spiketrain in segment.spiketrains:
            y = np.ones_like(spiketrain) * spiketrain.annotations['source_id']
            plt.plot(spiketrain, y, '.')
            plt.ylabel(segment.name)
            plt.setp(plt.gca().get_xticklabels(), visible=False)

    def plot_signal(signal, index, colour='b'):
        label = "Neuron %d" % signal.annotations['source_ids'][index]
        plt.plot(signal.times, signal[:, index], colour, label=label)
        plt.ylabel("%s (%s)" % (signal.name, signal.units._dimensionality.string))
        plt.setp(plt.gca().get_xticklabels(), visible=False)
        plt.legend()

    print("now plotting the network---------------")
    rospy.loginfo('--------now plotting---------------')
    n_panels = sum(a.shape[1] for a in readout_data.segments[0].analogsignalarrays) + 2
    plt.subplot(n_panels, 1, 1)
    plot_spiketrains(readout_data.segments[0])
    panel = 3
    for array in readout_data.segments[0].analogsignalarrays:
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
        network()
    except rospy.ROSInterruptException:
        pass
