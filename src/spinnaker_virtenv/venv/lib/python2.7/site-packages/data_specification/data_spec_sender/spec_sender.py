from spinnman.messages.sdp.sdp_message import SDPMessage
from spinnman.messages.sdp.sdp_header import SDPHeader
from spinnman.messages.sdp.sdp_flag import SDPFlag

import struct
import constants
import time


class SpecSender(object):
    """ Class for sending a data spec to a core.
        The data is added to a buffer using the add method and then sent using
        the send method.
    """

    __slots__ = [
        
        # The transceiver used to communicate with the board.
        "transceiver",

        # The destination of the data specification.
        "placement",

        # ???????????
        "header",

        # ???????????
        "msg_data"
    ]

    def __init__(self, transceiver, placement):
        """ Create a new SpecSender
        :param transceiver: The transceiver used to communicate with the board.
        :type transceiver: Transceiver
        :param placement: The destination of the data specification.
        :type placement: Placement
        :return:
        """
        self.transceiver = transceiver

        self.placement = placement

        self.header = SDPHeader(flags=SDPFlag.REPLY_NOT_EXPECTED,
                                destination_cpu=placement.p,
                                destination_chip_x=placement.x,
                                destination_chip_y=placement.y,
                                destination_port=1)
        self.msg_data = bytearray()

    def add(self, data):
        """ Add data to an internal buffer.
        :param data: The data to be added to the buffer.
        :type data: If bytearray, the internal buffer is just extended.
                    Otherwise, the data is converted to bytearray and added to
                    the buffer.
        :return:
        """
        if isinstance(data, bytearray):
            self.msg_data.extend(data)
        else:
            self.msg_data.extend(struct.pack("<I", data))

    def send(self):

        # Wait for the core to get into the READY_TO_RECEIVE state.
        while self.transceiver.get_cpu_information_from_core(
                self.placement.x, self.placement.y,
                self.placement.p).user[1] != constants.READY_TO_RECEIVE:
            time.sleep(0.01)
        # Send a packet containing the length of the data (the length of the
        # internal buffer).
        msg_data_len = struct.pack("<I", len(self.msg_data))

        self.transceiver.send_sdp_message(
            SDPMessage(self.header, msg_data_len))

        # Wait for the core to get into the WAITING_FOR_DATA state.
        while self.transceiver.\
            get_cpu_information_from_core(
                self.placement.x, self.placement.y,
                self.placement.p).user[1] != constants.WAITING_FOR_DATA:
            time.sleep(0.01)

        # Write data at the address pointed at by user2.
        destination_address = self.transceiver.get_cpu_information_from_core(
            self.placement.x, self.placement.y, self.placement.p).user[2]

        self.transceiver.write_memory(self.placement.x,
                                      self.placement.y,
                                      destination_address,
                                      self.msg_data,
                                      core_id=self.placement.p)

        # Send a packet that triggers the execution of the data specification.
        self.transceiver.send_sdp_message(SDPMessage(self.header,
                                                     struct.pack("<I", 0)))

        # Clear the internal buffer.
        self.msg_data = bytearray()
