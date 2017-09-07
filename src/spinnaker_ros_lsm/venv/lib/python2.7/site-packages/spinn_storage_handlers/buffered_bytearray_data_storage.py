import os
from spinn_storage_handlers.abstract_classes.abstract_buffered_data_storage \
    import AbstractBufferedDataStorage
from spinn_storage_handlers.exceptions import \
    BufferedBytearrayOperationNotImplemented


class BufferedBytearrayDataStorage(AbstractBufferedDataStorage):
    """ Data storage based on a bytearray buffer with two pointers,\
        one for reading and one for writing
    """

    def __init__(self):
        self._data_storage = bytearray()
        self._read_pointer = 0
        self._write_pointer = 0

    def write(self, data):
        if not isinstance(data, bytearray):
            raise
        if len(self._data_storage) == self._write_pointer:
            self._data_storage.extend(data)
        else:
            temp1 = self._data_storage[0:self._write_pointer]
            temp2 = self._data_storage[self._write_pointer:]
            temp1.extend(data)
            temp1.extend(temp2)
            self._data_storage = temp1
        self._write_pointer = len(self._data_storage)

    def read(self, data_size):
        start_id = self._read_pointer
        end_id = start_id + data_size
        self._read_pointer = end_id
        data = self._data_storage[start_id:end_id]
        return data

    def readinto(self, data):
        raise BufferedBytearrayOperationNotImplemented(
            "BufferedBytearrayDataStorage.readinto: operation unavailable")

    def read_all(self):
        return self._data_storage

    def seek_read(self, offset, whence=os.SEEK_SET):
        if whence == os.SEEK_SET:
            self._read_pointer = offset
        elif whence == os.SEEK_CUR:
            self._read_pointer += offset
        elif whence == os.SEEK_END:
            self._read_pointer = len(self._data_storage) - abs(offset)

        if self._read_pointer < 0:
            self._read_pointer = 0

        if self._read_pointer > len(self._data_storage):
            self._read_pointer = len(self._data_storage)

    def seek_write(self, offset, whence=os.SEEK_SET):
        if whence == os.SEEK_SET:
            self._write_pointer = offset
        elif whence == os.SEEK_CUR:
            self._write_pointer += offset
        elif whence == os.SEEK_END:
            self._write_pointer = len(self._data_storage) - abs(offset)

        if self._write_pointer < 0:
            self._write_pointer = 0

        if self._write_pointer > len(self._data_storage):
            self._write_pointer = len(self._data_storage)

    def tell_read(self):
        return self._read_pointer

    def tell_write(self):
        return self._write_pointer

    def eof(self):
        return (len(self._data_storage) - self._read_pointer) <= 0

    def close(self):
        self._data_storage = None
        self._read_pointer = 0
        self._write_pointer = 0
