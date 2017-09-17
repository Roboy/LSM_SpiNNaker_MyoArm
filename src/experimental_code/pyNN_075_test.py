
import pyNN.nest as p

celltype = p.IF_curr_exp

spike_source = p.native_cell_type('poisson_generator')
input_pop=p.Population(2,spike_source(rate))


