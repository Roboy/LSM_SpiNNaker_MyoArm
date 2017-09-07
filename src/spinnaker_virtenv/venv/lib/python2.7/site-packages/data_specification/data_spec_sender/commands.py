from enum import Enum
from data_specification_sender_functions \
    import DataSpecificationSenderFunctions as dssf


class Commands(Enum):
    """set of opcodes for the spec executor"""

    BREAK = (
        0x00, dssf.send_break, "Halts spec execution with an error")
    NOP = (
        0x01, dssf.send_nop, "No operation. Can be used as a filler")
    RESERVE = (
        0x02, dssf.send_reserve,
        "Reserves a block of memory ready for filling")
    WRITE = (
        0x41, dssf.send_write,
        "Performs a simple write or block write operation")
    WRITE_ARRAY = (
        0x42, dssf.send_write_array, "Performs a write from an array")
    SWITCH_FOCUS = (
        0x50, dssf.send_switch_focus,
        "Swap between different reserved memory regions to work on several"
        " at the same time")
    END_SPEC = (
        0XFF, dssf.send_end_spec, "Cleanly ends the parsing of the data specs")

    def __new__(cls, value, send_function, doc=""):
        obj = object.__new__(cls)
        obj._value_ = value
        obj.send_function = send_function
        obj.__doc__ = doc
        return obj

    def __init__(self, value, send_function, doc=""):
        self._value_ = value
        self.send_function = send_function
        self.__doc__ = doc
