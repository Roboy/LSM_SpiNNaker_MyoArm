# spinn front end common imports
from spinn_front_end_common.abstract_models.\
    abstract_provides_outgoing_partition_constraints import \
    AbstractProvidesOutgoingPartitionConstraints
from spinn_front_end_common.utility_models.multi_cast_command \
    import MultiCastCommand

# pynn imports
from spynnaker.pyNN.models.abstract_models\
    .abstract_send_me_multicast_commands_vertex \
    import AbstractSendMeMulticastCommandsVertex
from spynnaker.pyNN import exceptions

# pacman imports
from pacman.model.constraints.key_allocator_constraints\
    .key_allocator_fixed_key_and_mask_constraint \
    import KeyAllocatorFixedKeyAndMaskConstraint
from pacman.model.routing_info.base_key_and_mask import BaseKeyAndMask
from pacman.model.graphs.application.impl.application_spinnaker_link_vertex \
    import ApplicationSpiNNakerLinkVertex


# general imports
from collections import namedtuple
from enum import Enum, IntEnum

# Named tuple bundling together configuration elements of a pushbot resolution
# config
PushBotRetinaResolutionConfig = namedtuple("PushBotRetinaResolution",
                                           ["pixels", "enable_command",
                                            "coordinate_bits"])

PushBotRetinaResolution = Enum(
    value="PushBotRetinaResolution",
    names=[("Native128", PushBotRetinaResolutionConfig(128, (1 << 26), 7)),
           ("Downsample64", PushBotRetinaResolutionConfig(64, (2 << 26), 6)),
           ("Downsample32", PushBotRetinaResolutionConfig(32, (3 << 26), 5)),
           ("Downsample16", PushBotRetinaResolutionConfig(16, (4 << 26), 4))])

PushBotRetinaPolarity = IntEnum(
    value="PushBotRetinaPolarity",
    names=["Up", "Down", "Merged"])


class PushBotRetinaDevice(ApplicationSpiNNakerLinkVertex,
                          AbstractSendMeMulticastCommandsVertex,
                          AbstractProvidesOutgoingPartitionConstraints):

    # Mask for all SpiNNaker->Pushbot commands
    MANAGEMENT_MASK = 0xFFFFF800

    # Retina-specific commands
    RETINA_ENABLE = 0x1
    RETINA_DISABLE = 0x0
    RETINA_KEY_SET = 0x2
    RETINA_NO_TIMESTAMP = (0 << 29)

    # Sensor commands
    SENSOR = 0x7F0
    SENSOR_SET_KEY = 0x0
    SENSOR_SET_PUSHBOT = 0x1

    def __init__(
            self, fixed_key, spinnaker_link_id, label=None, n_neurons=None,
            polarity=PushBotRetinaPolarity.Merged,
            resolution=PushBotRetinaResolution.Downsample64,
            board_address=None):

        # Validate number of timestamp bytes
        if not isinstance(polarity, PushBotRetinaPolarity):
            raise exceptions.SpynnakerException(
                "Pushbot retina polarity should be one of those defined in"
                " Polarity enumeration")
        if not isinstance(resolution, PushBotRetinaResolution):
            raise exceptions.SpynnakerException(
                "Pushbot retina resolution should be one of those defined in"
                " Resolution enumeration")

        # Cache resolution
        self._resolution = resolution

        # Build standard routing key from virtual chip coordinates
        self._routing_key = fixed_key
        self._retina_source_key = self._routing_key

        # Calculate number of neurons
        fixed_n_neurons = resolution.value.pixels ** 2

        # If polarity is merged
        if polarity == PushBotRetinaPolarity.Merged:
            # Double number of neurons
            fixed_n_neurons *= 2

            # We need to mask out two coordinates and a polarity bit
            mask_bits = (2 * resolution.value.coordinate_bits) + 1
        # Otherwise
        else:
            # We need to mask out two coordinates
            mask_bits = 2 * resolution.value.coordinate_bits

            # If polarity is up, set polarity bit in routing key
            if polarity == PushBotRetinaPolarity.Up:
                polarity_bit = 1 << (2 * resolution.value.coordinate_bits)
                self._routing_key |= polarity_bit

        # Build routing mask
        self._routing_mask = ~((1 << mask_bits) - 1) & 0xFFFFFFFF

        ApplicationSpiNNakerLinkVertex.__init__(
            self, n_atoms=fixed_n_neurons, spinnaker_link_id=spinnaker_link_id,
            max_atoms_per_core=fixed_n_neurons, label=label,
            board_address=board_address)
        AbstractSendMeMulticastCommandsVertex.__init__(
            self, self._get_commands())
        AbstractProvidesOutgoingPartitionConstraints.__init__(self)

        if n_neurons != fixed_n_neurons and n_neurons is not None:
            print "Warning, the retina will have {} neurons".format(
                fixed_n_neurons)

    def get_outgoing_partition_constraints(self, partition):
        return [KeyAllocatorFixedKeyAndMaskConstraint(
            [BaseKeyAndMask(self._routing_key, self._routing_mask)])]

    def _get_commands(self):
        """
        method that returns the commands for the retina external device
        """
        # Set sensor key
        commands = list()
        commands.append(MultiCastCommand(
            0, PushBotRetinaDevice.SENSOR | PushBotRetinaDevice.SENSOR_SET_KEY,
            PushBotRetinaDevice.MANAGEMENT_MASK, self._retina_source_key,
            1, 100))

        # Set sensor to pushbot
        commands.append(MultiCastCommand(
            0, (PushBotRetinaDevice.SENSOR |
                PushBotRetinaDevice.SENSOR_SET_PUSHBOT),
            PushBotRetinaDevice.MANAGEMENT_MASK, 1,
            1, 100))

        # Ensure retina is disabled
        commands.append(MultiCastCommand(
            0, PushBotRetinaDevice.RETINA_DISABLE,
            PushBotRetinaDevice.MANAGEMENT_MASK, 0,
            1, 100))

        # Set retina key
        commands.append(MultiCastCommand(
            0, PushBotRetinaDevice.RETINA_KEY_SET,
            PushBotRetinaDevice.MANAGEMENT_MASK, self._retina_source_key,
            1, 100))

        # Enable retina
        commands.append(MultiCastCommand(
            0, PushBotRetinaDevice.RETINA_ENABLE,
            PushBotRetinaDevice.MANAGEMENT_MASK,
            (PushBotRetinaDevice.RETINA_NO_TIMESTAMP +
             self._resolution.value.enable_command),
            1, 100))

        # At end of simulation, disable retina
        commands.append(MultiCastCommand(
            -1, PushBotRetinaDevice.RETINA_DISABLE,
            PushBotRetinaDevice.MANAGEMENT_MASK, 0,
            1, 100))

        return commands

    @property
    def model_name(self):
        return "pushbot retina device"

    def recieves_multicast_commands(self):
        return True

    def is_virtual_vertex(self):
        return True
