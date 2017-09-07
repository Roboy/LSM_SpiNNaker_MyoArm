from spynnaker.pyNN.utilities import utility_calls
from spynnaker.pyNN.models.neural_properties.neural_parameter \
    import NeuronParameter
from data_specification.enums.data_type import DataType
from spynnaker.pyNN.models.neuron.threshold_types.abstract_threshold_type \
    import AbstractThresholdType

import numpy


class ThresholdTypeMaassStochastic(AbstractThresholdType):
    """ A stochastic threshold
    """

    def __init__(self, n_neurons, du_th, tau_th, v_thresh):
        AbstractThresholdType.__init__(self)
        self._n_neurons = n_neurons

        self._du_th = utility_calls.convert_param_to_numpy(du_th, n_neurons)
        self._tau_th = utility_calls.convert_param_to_numpy(tau_th, n_neurons)
        self._v_thresh = utility_calls.convert_param_to_numpy(
            v_thresh, n_neurons)

    @property
    def v_thresh(self):
        return self._v_thresh

    @v_thresh.setter
    def v_thresh(self, v_thresh):
        self._v_thresh = utility_calls.convert_param_to_numpy(
            v_thresh, self._n_neurons)

    @property
    def du_th(self):
        return self._du_th

    @du_th.setter
    def du_th(self, du_th):
        self._du_th = utility_calls.convert_param_to_numpy(
            du_th, self._n_neurons)

    @property
    def tau_th(self):
        return self._tau_th

    @tau_th.setter
    def tau_th(self, tau_th):
        self._tau_th = utility_calls.convert_param_to_numpy(
            tau_th, self._n_neurons)

    @property
    def _du_th_inv(self):
        return numpy.divide(1.0, self._du_th)

    @property
    def _tau_th_inv(self):
        return numpy.divide(1.0, self._tau_th)

    def get_n_threshold_parameters(self):
        return 3

    def get_threshold_parameters(self):
        return [
            NeuronParameter(self._du_th_inv, DataType.S1615),
            NeuronParameter(self._tau_th_inv, DataType.S1615),
            NeuronParameter(self._v_thresh, DataType.S1615)
        ]

    def get_n_cpu_cycles_per_neuron(self):
        return 30
