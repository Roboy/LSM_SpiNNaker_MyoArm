from spinn_storage_handlers.buffered_file_data_storage import \
    BufferedFileDataStorage
from spinn_storage_handlers.abstract_classes.abstract_data_reader import \
    AbstractDataReader


class FileDataReader(AbstractDataReader):
    """ A reader that can read data from a file
    """

    def __init__(self, filename):
        """
        :param filename: The file to read
        :type filename: str
        :raise spinn_storage_handlers.exceptions.DataReadException: If the\
                    file cannot found or opened for reading
        """
        self._file_container = BufferedFileDataStorage(filename, "rb")

    def read(self, n_bytes):
        """ See\
            :py:meth:`data_specification.abstract_data_reader.AbstractDataReader.read`
        """
        return self._file_container.read(n_bytes)

    def readall(self):
        """ See\
            :py:meth:`data_specification.abstract_data_reader.AbstractDataReader.readall`
        """
        return self._file_container.read_all()

    def readinto(self, data):
        """ See\
            :py:meth:`data_specification.abstract_data_reader.AbstractDataReader.readinto`
        """
        return self._file_container.readinto(data)

    def tell(self):
        """ Returns the position of the file cursor

        :return: Position of the file cursor
        :rtype: int
        """
        return self._file_container.tell_read()

    def close(self):
        """ Closes the file

        :return: Nothing is returned:
        :rtype: None
        :raise spinn_storage_handlers.exceptions.DataReadException: If the\
                    file cannot be closed
        """
        self._file_container.close()
