import pyNN.nest as p
from pyNN.random import NumpyRNG, RandomDistribution

import numpy as np
from scipy import signal

from common import (
spike_mean_rate, 
generate_labeledImages, 
print_mean_spike_rate, 
compute_weights,
compute_weights_exc_inh
)
from common import param

# All-to-all connections between input and reservoir
# Uniform weights between input and reservoir
# Uniform weights inside reservoir
# Excitatory and inhibitory neurons in reservor

# Issues: Negative weights does not take into account that 
# inhibitory neurons give negative potential

p.setup(timestep=param.dt)

######################


###### Neurons #######

input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson())
readout_neurons = p.Population(param.readout_nr, p.IF_curr_exp, {}, label="readout")

reservoir_exc = p.Population(param.res_exc_nr,p.IF_curr_exp, {}, label="reservoir_exc")
reservoir_inh = p.Population(param.res_inh_nr,p.IF_curr_exp, {}, label="reservoir_inh")

######################

###### Synapses #######

stat_syn_exc = p.StaticSynapse(weight =5.0, delay=1)
stat_syn_inh = p.StaticSynapse(weight =20.0, delay=1)
stat_syn_input = p.StaticSynapse(weight =50.0, delay=1)

######################

###### Connections #######

exc_conn = p.FixedProbabilityConnector(param.res_pconn, rng=param.rng)
inh_conn = p.FixedProbabilityConnector(param.res_pconn, rng=param.rng)
inp_conn = p.AllToAllConnector()
rout_conn = p.AllToAllConnector()

connections = {}
connections['e2e'] = p.Projection(reservoir_exc, reservoir_exc, exc_conn,
                                synapse_type=stat_syn_exc, receptor_type='excitatory')
connections['e2i'] = p.Projection(reservoir_exc, reservoir_inh, exc_conn,
                                synapse_type=stat_syn_exc,receptor_type='excitatory')
connections['i2e'] = p.Projection(reservoir_inh, reservoir_exc, inh_conn,
                                synapse_type=stat_syn_inh,receptor_type='inhibitory')
connections['i2i'] = p.Projection(reservoir_inh, reservoir_inh, inh_conn,
                                synapse_type=stat_syn_inh,receptor_type='inhibitory')

connections['inp2e'] = p.Projection(input_neurons, reservoir_exc, inp_conn,
                                      synapse_type=stat_syn_input,receptor_type='excitatory')
connections['inp2i'] = p.Projection(input_neurons, reservoir_inh, inp_conn,
                                      synapse_type=stat_syn_input,receptor_type='excitatory')

connections['e2rout'] = p.Projection(reservoir_exc, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_exc,receptor_type='excitatory')
connections['i2rout'] = p.Projection(reservoir_inh, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_inh,receptor_type='inhibitory')


######################

####### Feed images to network and record #######

input_neurons.record(['spikes'])
reservoir_exc.record(['spikes'])
reservoir_inh.record(['spikes'])
readout_neurons.record(['spikes'])

X = np.zeros( (param.images_train_nr,param.res_exc_nr+param.res_inh_nr) )
# Expected nr of spikes for readout neurons 
rout_left = [] #left images labels
rout_right = [] #right images labels
i = 0
for labeledImage in param.images_train:

	input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson, {'rate':labeledImage[0]})

	p.run(param.simulation_time)

	reservoir_exc_data = reservoir_exc.get_data(clear=True)
	reservoir_inh_data = reservoir_inh.get_data(clear=True)

	mean_rates = []
	for spiketrain in reservoir_exc_data.segments[0].spiketrains:
		mean_rate = spike_mean_rate(spiketrain, param.simulation_time)
		mean_rates.append(mean_rate)

	for spiketrain in reservoir_inh_data.segments[0].spiketrains:
		mean_rate = spike_mean_rate(spiketrain, param.simulation_time)
		mean_rates.append(mean_rate)

	X[i] = mean_rates

	rout_left.append(labeledImage[1][0])
	rout_right.append(labeledImage[1][1])

	i=i+1

print('Average spike matrix X')
print(X)


w_exc, w_inh = compute_weights_exc_inh(X, rout_left, rout_right)

connections['e2rout'].set(weight=w_exc)
connections['i2rout'].set(weight=w_inh)

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
