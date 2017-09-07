import numpy as np
from pyNN.random import NumpyRNG
from sklearn.linear_model import LinearRegression
import math

def gaussian_convolution(spikes,dt):
    #---- takes a spiketrain and the simulation time constant
    #     and computes the smoothed spike rate
    #-----works only after the simulation has run; not online!!!!!!!!
    kernel_size = 10
    gaussian_kernel = signal.gaussian(kernel_size, std=2)
    scaling_factor = 1/np.sum(gaussian_kernel)*1/dt
    gauss_rate = np.convolve(spikes,gaussian_kernel,mode='same')*scaling_factor
    mean_rate = np.mean(gauss_rate)
    return mean_rate

def spike_mean_rate(spikes, sim_time):
	return len(spikes) / sim_time

def generate_testImage(direction):
	potential = 100
	if direction=="left":
		return [potential,0,0,potential,0,0,potential,0,0]
	elif direction=='middle':
		return [0,potential,0,0,potential,0,0,potential,0]
	elif direction=='right':
		return [0,0,potential,0,0,potential,0,0,potential]
	else:
		return [0,0,0,0,0,0,0,0,0]

# Labeled image has the form (image, label)
# Label is a list [on1, on2], on# being the correct value for 
# the output neurons
def generate_labeledImages(nr):
	labeledImages = []
	for i in range(nr/3):
		labeledImages.append((generate_testImage("right"), [0,10]))
		labeledImages.append((generate_testImage("middle"), [0,0]))
		labeledImages.append((generate_testImage("left"), [10,0]))
		
	return labeledImages

# title: title of result
# strains: spiking trains
def print_mean_spike_rate(strains):
	mean_left = spike_mean_rate(strains[0], param.simulation_time)
	mean_right = spike_mean_rate(strains[1], param.simulation_time)
	print('Mean rate readout neurons (left, right)')
	print('(' + str(mean_left) + ',' + str(mean_right) + ')')

	return (mean_left, mean_right)

def compute_linear_weights(X, rout_left, rout_right):
	print('size of X',np.size(X))
	regr1 = LinearRegression()
	regr1.fit(X,rout_left)
	#print('Coefficients: \n', regr1.coef_)
	w1 = regr1.coef_
	
	regr2 = LinearRegression()
	regr2.fit(X,rout_right)
	#print('Coefficients: \n', regr2.coef_)
	w2 = regr2.coef_
		
	w = []
	for i in range(param.reservoir_nr):
		w.append(w1[i])
		w.append(w2[i])

	return w

def compute_weights(X, rout_left, rout_right):
	######### Fit weights to each output neuron with linear regression ###########

	w1 = np.linalg.lstsq(X.T.dot(X) + 0.1*np.identity(param.reservoir_nr), X.T.dot(rout_left))[0].tolist()

	# The coefficients
	print('Weights w1 reservoir - readout neuron left')
	print(w1)

	w2 = np.linalg.lstsq(X.T.dot(X) + 0.1*np.identity(param.reservoir_nr), X.T.dot(rout_right))[0].tolist()

	print('Weights w2 reservoir - readout neuron right')
	print(w2)

	# Connection['r2rout'] looks like
	# [ [r0, rout0, value], [r0, rout1, v], [r1, rout0, v] ... ]
	w = []
	for i in range(param.reservoir_nr):
		w.append(w1[i])
		w.append(w2[i])

	return w

def compute_weights_exc_inh(X, rout_left, rout_right):
	######### Fit weights to each output neuron with linear regression ###########

	w1 = np.linalg.lstsq(X.T.dot(X) + 0.1*np.identity(param.res_exc_nr+param.res_inh_nr), X.T.dot(rout_left))[0].tolist()

	# The coefficients
	print('Weights w1 reservoir - readout neuron left')
	print(w1)

	w2 = np.linalg.lstsq(X.T.dot(X) + 0.1*np.identity(param.res_exc_nr+param.res_inh_nr), X.T.dot(rout_right))[0].tolist()

	print('Weights w2 reservoir - readout neuron right')
	print(w2)

	# Connection['r2rout'] looks like
	# [ [r0, rout0, value], [r0, rout1, v], [r1, rout0, v] ... ]
	w = []
	for i in range(param.res_exc_nr+param.res_inh_nr):
		w.append(w1[i])
		w.append(w2[i])

	w_exc = []
	for i in range(param.res_exc_nr):
		w_exc.append(w1[i])
		w_exc.append(w2[i])

	w_inh = []
	for i in range(param.res_inh_nr):
		w_inh.append(w1[param.res_exc_nr + i])
		w_inh.append(w2[param.res_exc_nr + i])

	return (w_exc, w_inh)


class param:
	seed = 8658764			# Seed for reproduction of random number
	rng = NumpyRNG()		# Use seed to reproduce 
	input_nr = 9			# Number of input neurons
	readout_nr = 2			# Number of readout neurons
	reservoir_nr = 50		# Number of reservour neurons
	simulation_time = 19.0 # Simulation time for each input
	dt = 1					# Timestep in simulation
	res_pconn = 0.1			# sparse connection probability for reservoir
	images_train_nr = 9		# Number of training images to train with, 
							# Must be a factor of 3
	images_test_nr = 9  	# Number of test images
	images_train = generate_labeledImages(images_train_nr)
	images_test = generate_labeledImages(images_test_nr)

	# If network uses excitatory and inhibatory neurons
	res_exc_nr = int(math.ceil(reservoir_nr*0.8))		# Number of excitatory neurons
	res_inh_nr = int(math.floor(reservoir_nr*0.2))		# Number of inhibitory neurons

	print('exc:', res_exc_nr)






