from enum import Enum
import decimal


class DataType(Enum):
    """ Supported data types
        The first value is an identifier for the enum class;
        The second value is the size in bytes of the type;
        The third value is the minimum possible value for the type;
        The fourth value is the maximum possible value for the type;
        The fifth value is the scale of the input value to convert it in\
            integer;
        The sixth value is the pattern to use following the struct package\
            encodings to convert the data in binary format;
        The seventh value is the text description of the type.
    """
    UINT8 = (0,
             1,
             decimal.Decimal("0"),
             decimal.Decimal("255"),
             decimal.Decimal("1"),
             "B",
             "8-bit unsigned integer")
    UINT16 = (1,
              2,
              decimal.Decimal("0"),
              decimal.Decimal("65535"),
              decimal.Decimal("1"),
              "H",
              "16-bit unsigned integer")
    UINT32 = (2,
              4,
              decimal.Decimal("0"),
              decimal.Decimal("4294967295"),
              decimal.Decimal("1"),
              "I",
              "32-bit unsigned integer")
    UINT64 = (3,
              8,
              decimal.Decimal("0"),
              decimal.Decimal("18446744073709551615"),
              decimal.Decimal("1"),
              "Q",
              "64-bit unsigned integer")
    INT8 = (4,
            1,
            decimal.Decimal("-128"),
            decimal.Decimal("127"),
            decimal.Decimal("1"),
            "b",
            "8-bit signed integer")
    INT16 = (5,
             2,
             decimal.Decimal("-32768"),
             decimal.Decimal("32767"),
             decimal.Decimal("1"),
             "h",
             "16-bit signed integer")
    INT32 = (6,
             4,
             decimal.Decimal("-2147483648"),
             decimal.Decimal("2147483647"),
             decimal.Decimal("1"),
             "i",
             "32-bit signed integer")
    INT64 = (7,
             8,
             decimal.Decimal("-9223372036854775808"),
             decimal.Decimal("9223372036854775807"),
             decimal.Decimal("1"),
             "q",
             "64-bit signed integer")
    U88 = (8,
           2,
           decimal.Decimal("0"),
           decimal.Decimal("255.99609375"),
           decimal.Decimal("256"),
           "H",
           "8.8 unsigned fixed point number")
    U1616 = (9,
             4,
             decimal.Decimal("0"),
             decimal.Decimal("65535.9999847"),
             decimal.Decimal("65536"),
             "I",
             "16.16 unsigned fixed point number")
    U3232 = (10,
             8,
             decimal.Decimal("0"),
             decimal.Decimal("4294967295.99999999976716935634613037109375"),
             decimal.Decimal("4294967296"),
             "Q",
             "32.32 unsigned fixed point number")  # rounding problem for max
    S87 = (11,
           2,
           decimal.Decimal("-256"),
           decimal.Decimal("255.9921875"),
           decimal.Decimal("128"),
           "h",
           "8.7 signed fixed point number")
    S1615 = (12,
             4,
             decimal.Decimal("-32768"),
             decimal.Decimal("32767.9999847"),
             decimal.Decimal("32768"),
             "i",
             "16.15 signed fixed point number")
    S3231 = (13,
             8,
             decimal.Decimal("-4294967296"),
             decimal.Decimal("4294967295.9999999995343387126922607421875"),
             decimal.Decimal("2147483648"),
             "q",
             "32.31 signed fixed point number")  # rounding problem for max
    FLOAT_32 = (14,
                4,
                decimal.Decimal("-3.4028234e38"),
                decimal.Decimal("3.4028234e38"),
                decimal.Decimal("1"),
                "f",
                "32-bit floating point number")
    FLOAT_64 = (15,
                8,
                decimal.Decimal("-1.7976931348623157e+308"),
                decimal.Decimal("1.7976931348623157e+308"),
                decimal.Decimal("1"),
                "d",
                "64-bit floating point number")
    U08 = (16,
           1,
           decimal.Decimal("0"),
           decimal.Decimal("0.99609375"),
           decimal.Decimal("256"),
           "B",
           "0.8 unsigned fixed point number")
    U016 = (17,
            2,
            decimal.Decimal("0"),
            decimal.Decimal("0.999984741211"),
            decimal.Decimal("65536"),
            "H",
            "0.16 unsigned fixed point number")
    U032 = (18,
            4,
            decimal.Decimal("0"),
            decimal.Decimal("0.99999999976716935634613037109375"),
            decimal.Decimal("4294967296"),
            "I",
            "0.32 unsigned fixed point number")
    U064 = (19,
            8,
            decimal.Decimal("0"),
            decimal.Decimal(
                "0.9999999999999999999457898913757247782996273599565029"),
            decimal.Decimal("18446744073709551616"),
            "Q",
            "0.64 unsigned fixed point number")  # rounding problem for max
    S07 = (20,
           1,
           decimal.Decimal("-1"),
           decimal.Decimal("0.9921875"),
           decimal.Decimal("128"),
           "b",
           "0.7 signed fixed point number")
    S015 = (21,
            2,
            decimal.Decimal("-1"),
            decimal.Decimal("0.999969482421875"),
            decimal.Decimal("32768"),
            "h",
            "0.15 signed fixed point number")
    S031 = (22,
            4,
            decimal.Decimal("-1"),
            decimal.Decimal("0.99999999976716935634613037109375"),
            decimal.Decimal("2147483648"),
            "i",
            "0.32 signed fixed point number")
    S063 = (23,
            8,
            decimal.Decimal("-1"),
            decimal.Decimal(
                "0.9999999999999999998915797827514495565992547199130058"),
            decimal.Decimal("9223372036854775808"),
            "q",
            "0.63 signed fixed point number")  # rounding problem for max

    def __new__(cls, value, size, min_val, max_val, scale, struct_encoding,
                doc=""):
        obj = object.__new__(cls)
        obj._value_ = value
        obj.__doc__ = doc
        obj.size = size
        obj.min = min_val
        obj.max = max_val
        obj.scale = scale
        obj.struct_encoding = struct_encoding
        return obj

    def __init__(self, value, size, min_val, max_val, scale, struct_encoding,
                 doc=""):
        self._value_ = value
        self.__doc__ = doc
        self.size = size
        self.min = min_val
        self.max = max_val
        self.scale = scale
        self.struct_encoding = struct_encoding
