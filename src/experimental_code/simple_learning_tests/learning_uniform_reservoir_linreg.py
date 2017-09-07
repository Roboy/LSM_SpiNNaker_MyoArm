import pyNN.nest as p
from pyNN.random import NumpyRNG, RandomDistribution

import numpy as np
from scipy import signal

from common import (
spike_mean_rate, 
generate_labeledImages, 
print_mean_spike_rate, 
compute_weights
)
from common import param

# All-to-all connections between input and reservoir
# Uniform weights between input and reservoir
# Unifrom weights inside reservoir
# Uniform reservoir (all excitatory neurons)


p.setup(timestep=param.dt)

###### Neurons #######

input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson())
readout_neurons = p.Population(param.readout_nr, p.IF_curr_exp, {}, label="readout")
reservoir = p.Population(param.reservoir_nr,p.IF_curr_exp, {}, label="reservoir")

######################

###### Synapses #######

stat_syn_res = p.StaticSynapse(weight =5.0, delay=1)
stat_syn_input = p.StaticSynapse(weight =50.0, delay=1)
stat_syn_rout = p.StaticSynapse(weight =0.0, delay=1)

######################

###### Connections #######

res_conn = p.FixedProbabilityConnector(param.res_pconn, rng=param.rng)

inp_conn = p.AllToAllConnector()
rout_conn = p.AllToAllConnector()

connections = {}
connections['r2r'] = p.Projection(reservoir, reservoir, res_conn,
                                synapse_type=stat_syn_res, receptor_type='excitatory')

connections['inp2r'] = p.Projection(input_neurons, reservoir, inp_conn,
                                      synapse_type=stat_syn_input,receptor_type='excitatory')

connections['r2rout'] = p.Projection(reservoir, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_rout,receptor_type='excitatory')


######################

####### Feed images to network and record #######

input_neurons.record(['spikes'])
reservoir.record(['spikes'])
readout_neurons.record(['spikes'])

X = np.zeros( (param.images_train_nr,param.reservoir_nr) )
# Expected nr of spikes for readout neurons 
rout_left = [] #left images labels
rout_right = [] #right images labels
i = 0
for labeledImage in param.images_train:
	input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson, {'rate':labeledImage[0]})

	p.run(param.simulation_time)

	reservoir_data = reservoir.get_data(clear=True)

	mean_rates = []
	for spiketrain in reservoir_data.segments[0].spiketrains:
		mean_rate = spike_mean_rate(spiketrain, param.simulation_time)
		mean_rates.append(mean_rate)

	X[i] = mean_rates

	rout_left.append(labeledImage[1][0])
	rout_right.append(labeledImage[1][1])

	i=i+1


print('Average spike matrix X')
print(X)


w = compute_weights(X, rout_left, rout_right)
connections['r2rout'].set(weight=w)

######### Test accuracy ###########

print("\nTesting accuracy\n")

nr_correct = 0
for labeledImage in param.images_test:
	print('Image')
	print(labeledImage[0])
	input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson, {'rate':labeledImage[0]})
	p.run(param.simulation_time)

	readout_neurons_data = readout_neurons.get_data(clear=True)
	strains = readout_neurons_data.segments[0].spiketrains

	mean_left, mean_right = print_mean_spike_rate(strains)

	if labeledImage[1][0] > labeledImage[1][1]:
		if mean_left > mean_right:
			nr_correct = nr_correct + 1
	elif  labeledImage[1][1] > labeledImage[1][0]:
		if mean_right > mean_left:
			nr_correct = nr_correct + 1
	else:
		if abs(mean_left-mean_right) < 0.05:
			nr_correct = nr_correct + 1

acc = float(nr_correct) / param.images_test_nr
print("Accuracy: " + str(acc) + "%")