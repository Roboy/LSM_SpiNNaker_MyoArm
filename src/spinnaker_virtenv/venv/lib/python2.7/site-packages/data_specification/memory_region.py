
class MemoryRegion(object):
    """ Memory region storage object

    """

    __slots__ = [
        # the write pointer position
        "_mem_pointer",

        # flag that states if the region is filled or not
        "_unfilled",

        # the amount of memory allocated to this dsg
        "_allocated_size",

        # the region address map????
        "_region_data",

        # the position in the memory where the writing is currently occurring
        "_write_pointer",

        # the max point where if written over, it will cause an error
        "_max_write_pointer"
    ]

    def __init__(self, memory_pointer, unfilled, size):
        """

        :param memory_pointer: the write pointer position
        :param unfilled: if the region needs to be filled when written
        :return: None
        :rtype: None
        :raise None: this method does not raise any known exception
        """
        self._mem_pointer = memory_pointer
        self._unfilled = unfilled
        self._allocated_size = size
        self._region_data = bytearray(size)
        self._write_pointer = 0
        self._max_write_pointer = 0

    @property
    def memory_pointer(self):
        """ property method to retrieve memory write pointer

        :return: the memory pointer of the region
        :rtype: int
        :raise None: this method does not raise any known exception
        """
        return self._mem_pointer

    @property
    def allocated_size(self):
        """ property method for the size of the region

        :return: the size of the region
        :rtype: int
        :raise None: this method does not raise any known exception
        """
        return self._allocated_size

    @property
    def unfilled(self):
        """ property method to retrieve if the region is filled

        :return: the the
        :rtype: None
        :raise None: this method does not raise any known exception
        """
        return self._unfilled

    @property
    def region_data(self):
        """ the container which holds the data written in this region

        :return: the region data container
        :rtype: bytearray
        :raise None: this method does not raise any known exception
        """
        return self._region_data

    @property
    def write_pointer(self):
        return self._write_pointer

    @write_pointer.setter
    def write_pointer(self, write_pointer):
        self._write_pointer = write_pointer
        self._max_write_pointer = max((
            self._write_pointer, self._max_write_pointer))

    @property
    def max_write_pointer(self):
        return self._max_write_pointer

    def increment_write_pointer(self, n_bytes):
        self._write_pointer += n_bytes
        self._max_write_pointer = max((
            self._write_pointer, self._max_write_pointer))
