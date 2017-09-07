import os
from io import BlockingIOError

from spinn_storage_handlers.abstract_classes.abstract_buffered_data_storage \
    import AbstractBufferedDataStorage
from spinn_storage_handlers.exceptions import DataReadException, \
    DataWriteException


class BufferedFileDataStorage(AbstractBufferedDataStorage):
    """ Data storage based on a temporary file with two pointers, one for\
        reading and one for writing
    """

    __slots__ = [
        # ??????????????
        "_filename",

        # ??????????????
        "_file_size",

        # ??????????????
        "_read_pointer",

        # ??????????????
        "_write_pointer",

        # ?????????
        "_file"
    ]

    def __init__(self, filename, mode="rb"):
        self._filename = filename
        self._file_size = 0
        self._read_pointer = 0
        self._write_pointer = 0

        # open the file using the real handler
        try:
            self._file = open(filename, mode)
        except:
            raise DataReadException(
                "Unable to open file {}".format(filename))

    def write(self, data):
        if not (isinstance(data, bytearray) or isinstance(data, str)):
            raise DataWriteException(
                "BufferedFileDataStorage.write: Data to write is not in "
                "a suitable format. Current data format: "
                "{0:s}".format(type(data)))

        self._file.seek(self._write_pointer)

        try:
            self._file.write(data)
        except:
            raise IOError(
                "BufferedFileDataStorage.write: unable to write {0:d} "
                "bytes to file {1:s}".format(len(data), self._filename))

        self._file_size += len(data)
        self._write_pointer += len(data)

    def read(self, data_size):
        self._file.seek(self._read_pointer)

        try:
            data = self._file.read(data_size)
        except BlockingIOError:
            raise IOError(
                "BufferedFileDataStorage.read: unable to read {0:d} "
                "bytes from file {1:s}".format(data_size, self._filename))

        self._read_pointer += data_size
        return data

    def readinto(self, data):
        """ See :py:meth:`spinn_storage_handlers.abstract_classes.\
        abstract_buffered_data_storage.AbstractBufferedDataStorage.readinto`
        """
        self._file.seek(self._read_pointer)

        try:
            length = self._file.readinto(data)
        except BlockingIOError:
            raise IOError(
                "BufferedFileDataStorage.readinto: unable to read {0:d} bytes "
                "from file {1:s}".format(len(data), self._filename))

        self._read_pointer += length
        return length

    def read_all(self):
        self._file.seek(0)
        data = self._file.read()
        self._read_pointer = self._file.tell()
        return data

    def seek_read(self, offset, whence=os.SEEK_SET):
        if whence == os.SEEK_SET:
            self._read_pointer = offset
        elif whence == os.SEEK_CUR:
            self._read_pointer += offset
        elif whence == os.SEEK_END:
            self._read_pointer = self._file_size - abs(offset)

        if self._read_pointer < 0:
            self._read_pointer = 0

        file_len = self._file_len
        if self._read_pointer > file_len:
            self._read_pointer = file_len

    def seek_write(self, offset, whence=os.SEEK_SET):
        if whence == os.SEEK_SET:
            self._write_pointer = offset
        elif whence == os.SEEK_CUR:
            self._write_pointer += offset
        elif whence == os.SEEK_END:
            self._write_pointer = self._file_size - abs(offset)

        if self._write_pointer < 0:
            self._write_pointer = 0

        file_len = self._file_len
        if self._write_pointer > file_len:
            self._write_pointer = file_len

    def tell_read(self):
        return self._read_pointer

    def tell_write(self):
        return self._write_pointer

    def eof(self):
        file_len = self._file_len
        return (file_len - self._read_pointer) <= 0

    def close(self):
        try:
            self._file.close()
        except Exception:
            DataReadException(
                "BufferedFileDataStorage.close: File {} cannot "
                "be closed".format(self._filename))

    @property
    def _file_len(self):
        """ The size of the file

        :return: The size of the file
        :rtype: int
        """
        current_pos = self._file.tell()
        self._file.seek(0, 2)
        end_pos = self._file.tell()
        self._file.seek(current_pos)
        return end_pos

    @property
    def filename(self):
        """
        property method
        :return:
        """
        return self._filename
