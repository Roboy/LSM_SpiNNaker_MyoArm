from spynnaker.pyNN.utilities import utility_calls
from pacman.executor.injection_decorator import inject_items
from spynnaker.pyNN.models.neural_properties.neural_parameter \
    import NeuronParameter
from data_specification.enums.data_type import DataType
from spynnaker.pyNN.models.neuron.additional_inputs.abstract_additional_input \
    import AbstractAdditionalInput

import numpy


class AdditionalInputCa2Adaptive(AbstractAdditionalInput):

    def __init__(self, n_neurons, tau_ca2, i_ca2, i_alpha):
        AbstractAdditionalInput.__init__(self)

        self._n_neurons = n_neurons

        self._tau_ca2 = utility_calls.convert_param_to_numpy(
            tau_ca2, n_neurons)
        self._i_ca2 = utility_calls.convert_param_to_numpy(
            i_ca2, n_neurons)
        self._i_alpha = utility_calls.convert_param_to_numpy(
            i_alpha, n_neurons)

    @property
    def tau_ca2(self):
        return self._tau_ca2

    @tau_ca2.setter
    def tau_ca2(self, tau_ca2):
        self._tau_ca2 = utility_calls.convert_param_to_numpy(
            tau_ca2, self._n_neurons)

    @property
    def i_ca2(self):
        return self._i_ca2

    @i_ca2.setter
    def i_ca2(self, i_ca2):
        self._i_ca2 = utility_calls.convert_param_to_numpy(
            i_ca2, self._n_neurons)

    @property
    def i_alpha(self):
        return self._i_alpha

    @i_alpha.setter
    def i_alpha(self, i_alpha):
        self._i_alpha = utility_calls.convert_param_to_numpy(
            i_alpha, self._n_neurons)

    def _exp_tau_ca2(self, machine_time_step):
        return numpy.exp(float(-machine_time_step) /
                         (1000.0 * self._tau_ca2))

    def get_n_parameters(self):
        return 3

    @inject_items({"machine_time_step": "MachineTimeStep"})
    def get_parameters(self, machine_time_step):
        return [
            NeuronParameter(
                self._exp_tau_ca2(machine_time_step), DataType.S1615),
            NeuronParameter(self._i_ca2, DataType.S1615),
            NeuronParameter(self._i_alpha, DataType.S1615)
        ]

    def get_n_cpu_cycles_per_neuron(self):
        return 3

    def get_dtcm_usage_per_neuron_in_bytes(self):
        return 12

    def get_sdram_usage_per_neuron_in_bytes(self):
        return 12
