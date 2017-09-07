from pacman.model.decorators.overrides import overrides
from spinn_front_end_common.abstract_models.\
    abstract_provides_outgoing_partition_constraints import \
    AbstractProvidesOutgoingPartitionConstraints
from spinn_front_end_common.utility_models.reverse_ip_tag_multi_cast_source\
    import ReverseIpTagMultiCastSource

from pacman.model.constraints.key_allocator_constraints\
    .key_allocator_contiguous_range_constraint \
    import KeyAllocatorContiguousRangeContraint


class SpikeInjector(ReverseIpTagMultiCastSource,
                    AbstractProvidesOutgoingPartitionConstraints):
    """ An Injector of Spikes for PyNN populations.  This only allows the user\
        to specify the virtual_key of the population to identify the population
    """

    def __init__(self, n_neurons, label, port, virtual_key=None):

        ReverseIpTagMultiCastSource.__init__(
            self, n_keys=n_neurons, label=label, receive_port=port,
            virtual_key=virtual_key)

        AbstractProvidesOutgoingPartitionConstraints.__init__(self)

    @overrides(AbstractProvidesOutgoingPartitionConstraints.
               get_outgoing_partition_constraints)
    def get_outgoing_partition_constraints(self, partition):
        constraints = ReverseIpTagMultiCastSource\
            .get_outgoing_partition_constraints(self, partition)
        constraints.append(KeyAllocatorContiguousRangeContraint())
        return constraints
