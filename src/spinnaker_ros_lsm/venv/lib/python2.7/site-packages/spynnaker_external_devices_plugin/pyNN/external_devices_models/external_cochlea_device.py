from pacman.model.abstract_classes.abstract_virtual_vertex \
    import AbstractVirtualVertex


class ExternalCochleaDevice(AbstractVirtualVertex):

    def __init__(
            self, n_neurons, spinnaker_link, machine_time_step,
            timescale_factor, label=None):
        AbstractVirtualVertex.__init__(
            self, n_neurons, spinnaker_link, label=label,
            max_atoms_per_core=n_neurons)

    @property
    def model_name(self):
        return "ExternalCochleaDevice:{}".format(self.label)

    def is_virtual_vertex(self):
        return True
