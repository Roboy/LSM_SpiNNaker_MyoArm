from pacman.model.graphs.application.impl.application_edge \
    import ApplicationEdge
from spinnman.messages.eieio.eieio_type import EIEIOType
from spynnaker.pyNN import get_spynnaker
from spynnaker.pyNN.utilities import constants
from spinn_front_end_common.utility_models.live_packet_gather \
    import LivePacketGather


class SpynnakerExternalDevicePluginManager(object):
    """
    main entrance for the external device plugin manager
    """

    def __init__(self):
        self._live_spike_recorders = dict()

    @staticmethod
    def add_socket_address(socket_address):
        """ Add a socket address to the list to be checked by the\
            notification protocol

        :param socket_address: the socket address
        :type socket_address:
        :return:
        """
        _spinnaker = get_spynnaker()
        _spinnaker._add_socket_address(socket_address)

    def add_edge_to_recorder_vertex(
            self, vertex_to_record_from, port, hostname, tag=None,
            board_address=None,
            strip_sdp=True, use_prefix=False, key_prefix=None,
            prefix_type=None, message_type=EIEIOType.KEY_32_BIT,
            right_shift=0, payload_as_time_stamps=True,
            use_payload_prefix=True, payload_prefix=None,
            payload_right_shift=0, number_of_packets_sent_per_time_step=0):
        """
        adds a edge from a vertex to the LPG object, builds as needed and has
        all the parameters for the creation of the LPG if needed
        :param vertex_to_record_from:
        :param port:
        :param hostname:
        :param tag:
        :param board_address:
        :param strip_sdp:
        :param use_prefix:
        :param key_prefix:
        :param prefix_type:
        :param message_type:
        :param right_shift:
        :param payload_as_time_stamps:
        :param use_payload_prefix:
        :param payload_prefix:
        :param payload_right_shift:
        :param number_of_packets_sent_per_time_step:
        :return:
        """

        _spinnaker = get_spynnaker()

        # locate the live spike recorder
        if (port, hostname) in self._live_spike_recorders:
            live_spike_recorder = self._live_spike_recorders[(port, hostname)]
        else:

            live_spike_recorder = LivePacketGather(
                hostname, port, board_address, tag, strip_sdp, use_prefix,
                key_prefix, prefix_type, message_type, right_shift,
                payload_as_time_stamps, use_payload_prefix, payload_prefix,
                payload_right_shift, number_of_packets_sent_per_time_step,
                label="LiveSpikeReceiver")
            self._live_spike_recorders[(port, hostname)] = live_spike_recorder
            _spinnaker.add_application_vertex(live_spike_recorder)

        # create the edge and add
        edge = ApplicationEdge(
            vertex_to_record_from, live_spike_recorder, label="recorder_edge")
        _spinnaker.add_application_edge(edge, constants.SPIKE_PARTITION_ID)

    def add_edge(self, vertex, device_vertex, partition_id):
        """
        adds a edge between two vertices (often a vertex and a external device)
        on a given partition
        :param vertex: the pre vertex to connect the edge from
        :param device_vertex: the post vertex to connect the edge to
        :param partition_id: the partition identifier for making nets
        :return: None
        """
        _spinnaker = get_spynnaker()
        edge = ApplicationEdge(vertex, device_vertex)
        _spinnaker.add_application_edge(edge, partition_id)

    def machine_time_step(self):
        _spinnaker = get_spynnaker()
        return _spinnaker.machine_time_step

    def time_scale_factor(self):
        _spinnaker = get_spynnaker()
        return _spinnaker.timescale_factor
