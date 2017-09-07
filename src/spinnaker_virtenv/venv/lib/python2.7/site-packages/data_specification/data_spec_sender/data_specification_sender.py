import struct

from data_specification_sender_functions import \
    DataSpecificationSenderFunctions as Dssf
import constants
from commands import Commands
from command import Command


class DataSpecificationSender(object):
    """ Used to send a data specification language file to SpiNNaker to\
        produce a memory image
    """

    __slots__ = [
        # reader for the spec file
        "spec_reader",

        # Buffer used to send reliably data specification commands to the
        #  SpiNNaker board.
        "spec_sender",

        # report writer for human consumption
        "report_writer",

        # ?????????
        "dssf"
    ]

    def __init__(self, spec_reader, spec_sender, report_writer=None):
        """
        :param spec_reader: Reads from the DSG file.
        :type spec_reader: FileDataReader
        :param spec_sender: Buffer used to send reliably data specification\
                            commands to the SpiNNaker board.
        :type spec_sender: SpecSender
        :param report_writer: DSE report writer.
        :return:
        """
        self.spec_reader = spec_reader
        self.spec_sender = spec_sender
        self.report_writer = report_writer
        self.dssf = Dssf(self.spec_reader, self.spec_sender)

    def sendSpec(self):
        """ Parse the data specification form self.spec_reader, split it into\
            atomic chunks and send it using the spec_sender.
        """

        # read a new command
        instruction_spec = self.spec_reader.read(4)
        while len(instruction_spec) != 0:
            # process the received command
            cmd = Command(struct.unpack("<I", str(instruction_spec))[0])

            return_value = Commands(cmd.get_opcode()).send_function(
                self.dssf, cmd)
            self.spec_sender.send()

            if return_value == constants.END_SPEC_SENDER:
                break
            instruction_spec = self.spec_reader.read(4)
