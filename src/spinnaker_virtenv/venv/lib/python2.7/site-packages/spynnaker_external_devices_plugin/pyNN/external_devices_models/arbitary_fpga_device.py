# pacman imports
from pacman.model.graphs.application.impl.application_fpga_vertex \
    import ApplicationFPGAVertex

# general imports
from abc import ABCMeta
from six import add_metaclass


@add_metaclass(ABCMeta)
class ArbitaryFPGADevice(ApplicationFPGAVertex):

    def __init__(
            self, n_neurons, fpga_link_id, fpga_id, board_address=None,
            label=None):
        ApplicationFPGAVertex.__init__(
            self, n_neurons, fpga_id, fpga_link_id, board_address, label)
