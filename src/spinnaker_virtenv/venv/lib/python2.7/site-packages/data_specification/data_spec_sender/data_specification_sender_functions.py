import struct
import constants
from command import Command
from data_specification.data_spec_sender.commands import Commands


class DataSpecificationSenderFunctions(object):

    __slots__ = [
        
        # the reader for the spec file
        "spec_reader",

        # Buffer used to send reliably data specification commands to the
        #  SpiNNaker board.
        "spec_sender"
    ]

    def __init__(self, spec_reader, spec_sender):
        self.spec_reader = spec_reader
        self.spec_sender = spec_sender

    def send_break(self, cmd):
        self.spec_sender.add(cmd.get_value())

    def send_nop(self, cmd):
        self.spec_sender.add(cmd.get_value())

    def send_reserve(self, cmd):
        self.spec_sender.add(cmd.get_value())
        self.spec_sender.add(self.spec_reader.read(4))

    def send_write(self, cmd):
        self.spec_sender.add(cmd.get_value())
        for _ in range(cmd.get_length()):
            self.spec_sender.add(self.spec_reader.read(4))

    def send_write_array(self, cmd):
        self.spec_sender.add(cmd.get_value())

        length_encoded = self.spec_reader.read(4)
        self.spec_sender.add(length_encoded)

        length = struct.unpack("<I", str(length_encoded))[0]

        for _ in xrange(length):
            value_encoded = self.spec_reader.read(4)
            self.spec_sender.add(value_encoded)

    def send_switch_focus(self, cmd):
        self.spec_sender.add(cmd.get_value())

    def send_end_spec(self, cmd):
        self.spec_sender.add(cmd.get_value())
        return constants.END_SPEC_SENDER

    def send_loop(self, cmd):
        self.spec_sender.add(cmd.get_value())
        while cmd.get_opcode() != Commands.END_LOOP:
            cmd = Command(self.spec_reader.read(4))
            self.spec_sender.add(cmd.get_value())
