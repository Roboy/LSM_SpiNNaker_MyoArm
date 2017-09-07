class DataWriteException(Exception):
    """ An exception that indicates that there was an error writing\
        to the underlying medium
    """

    def __init__(self, message):
        """

        :param message: A message to indicate what when wrong
        :type message: str
        """
        Exception.__init__(self, message)


class DataReadException(Exception):
    """ An exception that indicates that there was an error reading\
        from the underlying medium
    """

    def __init__(self, message):
        """

        :param message: A message to indicate what when wrong
        :type message: str
        """
        Exception.__init__(self, message)


class BufferedBytearrayOperationNotImplemented(Exception):
    """ An exception that denotes that the operation required is unavailable
        for a byteArray buffer
    """
    def __init__(self, message):
        """

        :param message: A message to indicate what when wrong
        :type message: str
        """
        Exception.__init__(self, message)
