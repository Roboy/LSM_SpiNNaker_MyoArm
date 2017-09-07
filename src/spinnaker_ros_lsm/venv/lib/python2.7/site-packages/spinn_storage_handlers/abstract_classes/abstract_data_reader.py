from abc import ABCMeta
from abc import abstractmethod
from six import add_metaclass


@add_metaclass(ABCMeta)
class AbstractDataReader(object):
    """ Abstract reader used to read data from somewhere
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
    def read(self, n_bytes):
        """ Read some bytes of data from the underlying storage.  Will block\
            until some bytes are available, but might not return the full\
            n_bytes.  The size of the returned array indicates how many\
            bytes were read.
        
        :param n_bytes: The number of bytes to read
        :type n_bytes: int
        :return: An array of bytes
        :rtype: bytearray
        :raise IOError: If an error occurs reading from the underlying storage
        """
        pass
    
    @abstractmethod
    def readinto(self, data):
        """ Read some bytes of data from the underlying storage into a\
            pre-defined array.  Will block until some bytes are available,\
            but may not fill the array completely.
        
        :param data: The place where the data is to be stored
        :type data: bytearray
        :return: The number of bytes stored in data
        :rtype: int
        :raise IOError: If an error occurs reading from the underlying storage
        """
        pass

    @abstractmethod
    def readall(self):
        """ Read the rest of the bytes from the underlying stream

        :return: The bytes read
        :rtype: bytearray
        :raise IOError: If there is an error obtaining the bytes
        """
        pass

    @abstractmethod
    def tell(self):
        """ Returns the position of the file cursor

        :return: Position of the file cursor
        :rtype: int
        """
        pass
