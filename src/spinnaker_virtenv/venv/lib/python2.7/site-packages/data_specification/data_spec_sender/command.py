class Command(object):

    __slots__ = [
        # value of the command
        '_value'
    ]

    def __init__(self, cmd):
        self._value = cmd

    def get_value(self):
        return self._value

    def get_length(self):
        return (self._value >> 28) & 0x3

    def get_opcode(self):
        return (self._value >> 20) & 0xFF
