from enum import Enum


class LogicOperation(Enum):
    """ Logic Operations
    """
    LEFT_SHIFT = (0, "LSL", "Shift left")
    RIGHT_SHIFT = (1, "LSR", "Shift right")
    OR = (2, "OR", "Logical OR")
    AND = (3, "AND", "Logical AND")
    XOR = (4, "XOR", "Logical XOR")
    NOT = (5, "NOT", "Logical NOT")

    def __new__(cls, value, operator, doc=""):
        obj = object.__new__(cls)
        obj._value_ = value
        obj.operator = operator
        obj.__doc__ = doc
        return obj

    def __init__(self, value, operator, doc=""):
        self._value_ = value
        self.operator = operator
        self.__doc__ = doc
