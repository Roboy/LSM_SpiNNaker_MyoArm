from abc import ABCMeta
from abc import abstractmethod
from six import add_metaclass


@add_metaclass(ABCMeta)
class AbstractDataWriter(object):
    """ Abstract writer used to write data somewhere
    """

    @classmethod
    def __subclasshook__(cls, othercls):
        """ Checks if all the abstract methods are present on the subclass
        """
        for C in cls.__mro__:
            for key in C.__dict__:
                item = C.__dict__[key]
                if hasattr(item, "__isabstractmethod__"):
                    if not any(key in B.__dict__ for B in othercls.__mro__):
                        return NotImplemented
        return True

    @abstractmethod
    def write(self, data):
        """ Write some bytes of data to the underlying storage.\
            Does not return until all the bytes have been written.

        :param data: The data to write
        :type data: bytearray
        :return: Nothing is returned
        :rtype: None
        :raise IOError: If an error occurs writing to the underlying storage
        """
        pass

    @abstractmethod
    def tell(self):
        """ Returns the position of the file cursor

        :return: Position of the file cursor
        :rtype: int
        """
        pass
