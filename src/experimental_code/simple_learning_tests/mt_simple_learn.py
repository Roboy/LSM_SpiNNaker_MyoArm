##%%
import pyNN.nest as p 
from pyNN.random import NumpyRNG, RandomDistribution

import numpy as np 


# input: trajectory
# liquid: representation of current value (and last ones)
# output: 
input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson())
readout_neurons = p.Population(param.readout_nr, p.IF_curr_exp, {}, label="readout")
reservoir = p.Population(param.reservoir_nr,p.IF_curr_exp, {}, label="reservoir")

