"""
The :py:mod:`spynnaker.pynn` package contains the front end specifications
and implementation for the PyNN High-level API
(http://neuralensemble.org/trac/PyNN)
"""

import os

from spinnman.messages.eieio.eieio_type import EIEIOType
from spynnaker.pyNN.utilities import constants
from spynnaker_external_devices_plugin.pyNN.external_devices_models.\
    external_spinnaker_link_cochlea_device import ExternalCochleaDevice
from spynnaker_external_devices_plugin.pyNN.external_devices_models.\
    external_spinnaker_link_fpga_retina_device import ExternalFPGARetinaDevice
from spynnaker_external_devices_plugin.pyNN.external_devices_models.\
    munich_spinnaker_link_retina_device import MunichRetinaDevice
from spynnaker_external_devices_plugin.pyNN.external_devices_models.\
    pushbot_spinnaker_link_retina_device import PushBotRetinaDevice
from spynnaker_external_devices_plugin.pyNN.external_devices_models.\
    pushbot_spinnaker_link_retina_device import PushBotRetinaResolution
from spynnaker_external_devices_plugin.pyNN.external_devices_models.\
    pushbot_spinnaker_link_retina_device import PushBotRetinaPolarity
from spynnaker_external_devices_plugin.pyNN.external_devices_models.\
    munich_spinnaker_link_motor_device import MunichMotorDevice
from spynnaker_external_devices_plugin.pyNN.external_devices_models.\
    arbitary_fpga_device import ArbitaryFPGADevice

from spynnaker_external_devices_plugin.pyNN import model_binaries

from spynnaker_external_devices_plugin.pyNN.\
    spynnaker_external_device_plugin_manager import \
    SpynnakerExternalDevicePluginManager
from spynnaker_external_devices_plugin.pyNN.utility_models.spike_injector \
    import SpikeInjector as SpynnakerExternalDeviceSpikeInjector
from spynnaker_external_devices_plugin.pyNN.connections\
    .spynnaker_live_spikes_connection import SpynnakerLiveSpikesConnection


from spynnaker.pyNN.utilities import conf
from spynnaker.pyNN.spinnaker import executable_finder


from spinn_front_end_common.utilities.notification_protocol.socket_address \
    import SocketAddress

executable_finder.add_path(os.path.dirname(model_binaries.__file__))
spynnaker_external_devices = SpynnakerExternalDevicePluginManager()


def activate_live_output_for(
        population, database_notify_host=None, database_notify_port_num=None,
        database_ack_port_num=None, board_address=None, port=None,
        host=None, tag=None, strip_sdp=True, use_prefix=False, key_prefix=None,
        prefix_type=None, message_type=EIEIOType.KEY_32_BIT,
        right_shift=0, payload_as_time_stamps=True,
        use_payload_prefix=True, payload_prefix=None,
        payload_right_shift=0, number_of_packets_sent_per_time_step=0):
    """ Output the spikes from a given population from SpiNNaker as they
        occur in the simulation

    :param population: The population to activate the live output for
    :type population: Population
    :param database_notify_host: the hostname for the device which is\
            listening to the database notification.
    :type database_notify_host: str
    :param database_ack_port_num: the port number to which a external device\
            will acknowledge that they have finished reading the database and\
            are ready for it to start execution
    :type database_ack_port_num: int
    :param database_notify_port_num: The port number to which a external\
            device will receive the database is ready command
    :type database_notify_port_num: int
    :param board_address: A fixed board address required for the tag, or\
            None if any address is OK
    :type board_address: str
    :param key_prefix: the prefix to be applied to the key
    :type key_prefix: int or None
    :param prefix_type: if the prefix type is 32 bit or 16 bit
    :param message_type: if the message is a eieio_command message, or\
            eieio data message with 16 bit or 32 bit keys.
    :param payload_as_time_stamps:
    :param right_shift:
    :param use_payload_prefix:
    :param payload_prefix:
    :param payload_right_shift:
    :param number_of_packets_sent_per_time_step:

    :param port: The UDP port to which the live spikes will be sent.  If not\
                specified, the port will be taken from the "live_spike_port"\
                parameter in the "Recording" section of the spynnaker cfg file.
    :type port: int
    :param host: The host name or IP address to which the live spikes will be\
                sent.  If not specified, the host will be taken from the\
                "live_spike_host" parameter in the "Recording" section of the\
                spynnaker cfg file.
    :type host: str
    :param tag: The IP tag to be used for the spikes.  If not specified, one\
                will be automatically assigned
    :type tag: int
    :param strip_sdp: Determines if the SDP headers will be stripped from the\
                transmitted packet.
    :type strip_sdp: bool
    :param use_prefix: Determines if the spike packet will contain a common\
                prefix for the spikes
    :type use_prefix: bool
    """

    # get default params if none set
    if port is None:
        port = conf.config.getint("Recording", "live_spike_port")
    if host is None:
        host = conf.config.get("Recording", "live_spike_host")
    # get default params for the database socket if required

    if database_notify_port_num is None:
        database_notify_port_num = conf.config.getint("Database",
                                                      "notify_port")
    if database_notify_host is None:
        database_notify_host = conf.config.get("Database", "notify_hostname")
    if database_ack_port_num is None:
        database_ack_port_num = conf.config.get("Database", "listen_port")
        if database_ack_port_num == "None":
            database_ack_port_num = None

    # add new edge and vertex if required to spinnaker graph
    spynnaker_external_devices.add_edge_to_recorder_vertex(
        population._vertex, port, host, tag, board_address, strip_sdp,
        use_prefix, key_prefix, prefix_type, message_type, right_shift,
        payload_as_time_stamps, use_payload_prefix, payload_prefix,
        payload_right_shift, number_of_packets_sent_per_time_step)
    # build the database socket address used by the notification interface
    database_socket = SocketAddress(
        listen_port=database_ack_port_num,
        notify_host_name=database_notify_host,
        notify_port_no=database_notify_port_num)
    # update socket interface with new demands.
    spynnaker_external_devices.add_socket_address(database_socket)


def activate_live_output_to(population, device):
    """ Activate the output of spikes from a population to an external device.\
        Note that all spikes will be sent to the device.

    :param population: The pyNN population object from which spikes will be\
                sent.
    :param device: The pyNN population external device to which the spikes\
                will be sent.
    """
    spynnaker_external_devices.add_edge(
        population._get_vertex, device._get_vertex,
        constants.SPIKE_PARTITION_ID)


def SpikeInjector(
        n_neurons, label, port,
        virtual_key=None, database_notify_host=None,
        database_notify_port_num=None, database_ack_port_num=None):
    """ Supports adding a spike injector to the application graph.

    :param n_neurons: the number of neurons the spike injector will emulate
    :type n_neurons: int
    :param label: the label given to the population
    :type label: str
    :param port: the port number used to listen for injections of spikes
    :type port: int
    :param virtual_key: the virtual key used in the routing system
    :type virtual_key: int
    :param database_notify_host: the hostname for the device which is\
            listening to the database notification.
    :type database_notify_host: str
    :param database_ack_port_num: the port number to which a external device\
            will acknowledge that they have finished reading the database and\
            are ready for it to start execution
    :type database_ack_port_num: int
    :param database_notify_port_num: The port number to which a external\
            device will receive the database is ready command
    :type database_notify_port_num: int

    :return:
    """
    if database_notify_port_num is None:
        database_notify_port_num = conf.config.getint("Database",
                                                      "notify_port")
    if database_notify_host is None:
        database_notify_host = conf.config.get("Database", "notify_hostname")
    if database_ack_port_num is None:
        database_ack_port_num = conf.config.get("Database", "listen_port")
        if database_ack_port_num == "None":
            database_ack_port_num = None

    # build the database socket address used by the notification interface
    database_socket = SocketAddress(
        listen_port=database_ack_port_num,
        notify_host_name=database_notify_host,
        notify_port_no=database_notify_port_num)

    # update socket interface with new demands.
    spynnaker_external_devices.add_socket_address(database_socket)
    return SpynnakerExternalDeviceSpikeInjector(
        n_neurons=n_neurons, label=label, port=port, virtual_key=virtual_key)
