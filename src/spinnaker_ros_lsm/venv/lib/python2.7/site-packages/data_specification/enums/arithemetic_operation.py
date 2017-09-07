from enum import Enum


class ArithmeticOperation(Enum):
    """ Arithmetic Operations
    """

    ADD = (0, "+", "Perform addition")
    SUBTRACT = (1, "-", "Perform subtraction")
    MULTIPLY = (2, "*", "Perform multiplication")

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
