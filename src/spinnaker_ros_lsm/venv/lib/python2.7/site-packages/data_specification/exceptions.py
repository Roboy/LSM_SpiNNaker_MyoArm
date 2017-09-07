class DataSpecificationException(Exception):
    """ A general purpose exception indicating that something went\
        wrong when interacting with a Data Specification
    """

    def __init__(self, message):
        """

        :param message: message reporting the exception occurred
        :type message: str
        """
        Exception.__init__(self, message)


class DataUndefinedWriterException(Exception):
    """an exception that indicates that the file data writer has not been\
       initialised
    """

    def __init__(self, message):
        """

        :param message: A message to indicate what when wrong
        :type message: str
        """
        Exception.__init__(self, message)


class DataSpecificationRegionInUseException(DataSpecificationException):
    """ An exception that indicates that a region has already been\
        allocated
    """

    def __init__(self, region):
        """

        :param region: The region that was already allocated
        :type region: int
        """
        Exception.__init__(
            self, "Region {0:d} was already allocated".format(region))


class DataSpecificationStructureInUseException(DataSpecificationException):
    """ An exception that indicates that a structure has already been\
        defined
    """

    def __init__(self, structure):
        """

        :param structure: The structure that was already allocated
        :type structure: int
        """
        Exception.__init__(
            self, "Structure {0:d} was already allocated".format(
                structure))


class DataSpecificationRegionUnfilledException(DataSpecificationException):
    """ An exception that indicates that a memory region is being used\
        that was originally requested to be unfilled
    """

    def __init__(self, region, command):
        """

        :param region: The region that was requested as unfilled
        :type region: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "Region {0:d} was requested unfilled, but command {1:s} "
            "requests its use".format(region, command))


class DataSpecificationNoRegionSelectedException(DataSpecificationException):
    """ An exception that indicates that a memory region has not been selected
    """

    def __init__(self, command):
        """

        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "Command {0:s} tries to operate on an unspecified memory "
            "region".format(command))


class DataSpecificationRegionExhaustedException(DataSpecificationException):
    """ An exception that indicates that a region has run out of memory\
        whilst some data is being written
    """

    def __init__(self, region, region_size, allocated_size, command):
        """

        :param region: The region that was being written to
        :type region: int
        :param region_size: The originally requested size of the region that\
                    has run out of space, in bytes
        :type region_size: int
        :param allocated_size: The amount of the originally requested space\
                    that has already been allocated, in bytes
        :type allocated_size: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "Region {0:d} with size {1:d} ran out of allocated memory "
            "(space already occupied {2:d}) during command {3:s}".format(
                region, region_size, allocated_size, command))


class DataSpecificationRegionOutOfBoundsException(DataSpecificationException):
    """ An exception that indicates that an offset into a region is out\
        of bounds for that region
    """

    def __init__(self, region, region_size, requested_offset, command):
        """

        :param region: The region that was being offset into
        :type region: int
        :param region_size: The originally requested size of the region in\
                    question, in bytes
        :type region_size: int
        :param requested_offset: The offset being requested, in bytes
        :type requested_offset: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "Requesting offset {0:d} into region {1:d} with size {2:d} "
            "during command {3:s}".format(
                requested_offset, region, region_size, command))


class DataSpecificationParameterOutOfBoundsException(
        DataSpecificationException):
    """ An exception that indicates that a parameter value was outside of the\
        allowed bounds
    """

    def __init__(self, parameter, value, range_min, range_max, command):
        """

        :param parameter: The parameter that is out of bounds
        :type parameter: str
        :param value: The value specified
        :type value: float
        :param range_min: The minimum allowed value
        :type range_min: float
        :param range_max: The maximum allowed value
        :type range_max: float
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self,
            "Requesting value {} for parameter {} whose allowed range "
            "is from {} to {} during command {}".format(
                value, parameter, range_min, range_max, command))


class DataSpecificationNotAllocatedException(DataSpecificationException):
    """ An exception that indicates that an item is being used\
        that has not been allocated
    """

    def __init__(self, item_type, item_id, command):
        """

        :param item_type: The type of the item being used
        :type item_type: str
        :param item_id: The id of the item being used
        :type item_id: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "Using unallocated item with type {0:s} and id {1:d} during "
            "command {2:s}".format(item_type, item_id, command))


class DataSpecificationNoMoreException(DataSpecificationException):
    """ An exception that indicates that there is no more space for the\
        requested item
    """

    def __init__(self, space_available, space_required, region):
        """

        :param space_available: The space available in the region
        :type space_available: int
        :param space_required: The space requested by the write command
        :type space_required: int
        """
        Exception.__init__(
            self,
            "Space unavailable to write all the elements requested by the "
            "write operation. Space available: {}; space requested: "
            "{} for region {}.".format(
                space_available, space_required, region))


class DataSpecificationFunctionInUse(DataSpecificationException):
    """ An exception that indicates that a function is already defined
    """

    def __init__(self, function_id):
        """
        :param function_id: The id of the function
        :type function_id:int
        """
        Exception.__init__(
            self, "Function {0:d} is already defined".format(function_id))


class DataSpecificationRNGInUseException(DataSpecificationException):
    """ An exception that indicates that a random number generator is\
        already defined
    """

    def __init__(self, rng_id):
        """
        :param rng_id: The id of the rng
        :type rng_id:int
        """
        Exception.__init__(
            self,
            "Random number generator {0:d} is already defined".format(rng_id))


class DataSpecificationRandomNumberDistributionInUseException(
        DataSpecificationException):
    """ An exception that indicates that a random number distribution is\
        already defined
    """

    def __init__(self, rng_id):
        """
        :param id: The id of the random number distribution
        :type id: int
        """
        Exception.__init__(
            self,
            "Random number distribution {0:d} is already defined".format(
                rng_id))


class DataSpecificationWrongParameterNumberException(
        DataSpecificationException):
    """ An exception that indicates that a function has been called with a\
        wrong number of parameters.
    """

    def __init__(self, function_id, no_of_parameters_required, parameters):
        """
        :param function_id: The id of the function
        :type function_id: int
        :param parameters: The parameters used in the function call
        :type parameters: list
        :param no_of_parameters_required: The number of parameters required by
                                          the function
        """
        Exception.__init__(
            self,
            "Function {0:d} that requires {1:d} parameters has been called "
            "with the following parameters: {2:s}".format(
                function_id, no_of_parameters_required, parameters))


class DataSpecificationDuplicateParameterException(DataSpecificationException):
    """ And exception that indicates that a command has been called with a\
        duplicate parameter, which shouldn't be allowed.
    """

    def __init__(self, command, parameters):
        """
        :param command: The command called with duplicate parameters
        :type command: int
        :param parameters: The parameters used to call the function
        :type parameters: list
        """

        Exception.__init__(
            self,
            "The command {0:s} has been called with duplicate parameters: "
            "{1:s}".format(command, repr(parameters)))


class NestedFunctionException(DataSpecificationException):
    """ An exception that indicates that a function is being defined within\
        the context of another function definition
    """

    def __init__(self):
        """

        """
        Exception.__init__(self, "Nested function definition not supported")


class DataSpecificationTypeMismatchException(DataSpecificationException):
    """ An exception that indicates that a type mismatch has occurred
    """

    def __init__(self, command):
        """
        :param command: The command that generated the exception
        :type command: int
        """
        Exception.__init__(
            self, "A type mismatch has occurred during command {0:s}".format(
                command))


class DataSpecificationUnknownTypeException(DataSpecificationException):
    """ An exception that indicates that the value of the requested type\
        is unknown
    """

    def __init__(self, type_id, command):
        """

        :param type_id: The id of the requested type
        :type type_id: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "Unknown id value {0:d} for data type during command "
            "{1:s}".format(type_id, command))


class DataSpecificationUnknownTypeLengthException(DataSpecificationException):
    """ An exception that indicates that the value of the requested type\
        is unknown
    """

    def __init__(self, data_length, command):
        """

        :param data_length: the length of the requested type
        :type data_length: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "Unknown data length {0:d} during command {1:s}".format(
                data_length, command))


class DataSpecificationInvalidSizeException(DataSpecificationException):
    """ An exception that indicates that the size of the requested type is\
        invalid
    """

    def __init__(self, type_name, type_size, command):
        """

        :param type_name: The name of the requested type
        :type type_name: str
        :param type_size: The size of the requested variable
        :type type_size: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "Invalid size {0:d} of the requested type {1:s} during "
            "command {2:s}".format(type_size, type_name, command))


class DataSpecificationInvalidCommandException(DataSpecificationException):
    """ An exception that indicates that the command being requested cannot be
        executed at this point in the specification
    """

    def __init__(self, command):
        """

        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self,
            "The requested command {0:s} cannot be executed at this point "
            "in the specification".format(command))


class DataSpecificationUnknownConditionException(DataSpecificationException):
    """ And exception which is triggered in case the condition in an IF test
     does not exist in the list of possible conditions
    """

    def __init__(self, condition_id, command):
        """

        :param condition_id: id of the condition being requested
        :type condition_id: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self,
            "The requested condition with id {0:d} does not belong to the "
            "list of possible tests during command {1:s}".format(
                condition_id, command))


class DataSpecificationInvalidOperationException(DataSpecificationException):
    """ An exception that indicates that the operation of the type given type
    is not available
    """

    def __init__(self, operation_type, requested_operation_id, command):
        """

        :param operation_type: The type of operation requested \
                    (i.e. arithmetic or logic)
        :type operation_type: str
        :param requested_operation_id: The id of the requested operation
        :type requested_operation_id: int
        :param command: The command being executed
        :type command: str
        """
        Exception.__init__(
            self, "The {0:s} operation requested with id {1:d} does not match "
            "the possible operations available during command {2:s}".format(
                operation_type, requested_operation_id, command))


class ExecuteBreakInstruction(DataSpecificationException):
    """ An exception which occurs when a BREAK instruction is found in the\
        data specification
    """

    def __init__(self, address, filename):
        """

        :param address: address of the data specification being executed\
                    at the time of breakpoint
        :type address: int
        :param filename: file being parsed
        :param filename: str
        """
        Exception.__init__(
            self, "Executing BREAK instruction at address {0:d} of file "
            "{1:s}".format(address, filename))


class DataSpecificationSyntaxError(DataSpecificationException):
    """ An exception which occurs when a command read from the data\
        specification file shows an inconsistency in the binary content
    """

    def __init__(self, message):
        """

        :param message: message describing the error occurred
        :type message: str
        """
        Exception.__init__(self, message)


class DataSpecificationTablePointerOutOfMemory(DataSpecificationException):
    """ An exception which occurs when building the table pointer as header of\
        the data generated by the spec executor. This message is printed in\
        case the memory available is not enough to contain the pointer table\
        for each of the allocated regions
    """

    def __init__(self, memory_available, memory_required):
        """

        :param memory_available: on-chip memory available
        :type memory_available: int
        :param memory_required: on-chip memory required to complete the
        execution of the specification file
        :type memory_required: int
        """
        Exception.__init__(
            self,
            "The memory available {0:d} is not sufficient for the allocated"
            " regions plus the header table pointer {1:d}".format(
                memory_available, memory_required))


class DataSpecificationRegionNotAllocated(DataSpecificationException):
    """ An exception which occurs when trying to write to an unallocated\
        region of memory
    """

    def __init__(self, region, command):
        """

        :param region:
        :type region:
        :param command:
        :type command:
        """
        Exception.__init__(
            self, "Region {0:d} has not been allocated during execution of "
            "command {1:s}".format(region, command))


class UnimplementedDSECommand(DataSpecificationException):
    """ An exception which occurs when trying to execute an unimplemented\
        command
    """

    def __init__(self, command):
        """

        :param command: Command attempted to be executed by the DSE
        :type command: str
        """
        Exception.__init__(
            self,
            "Command {0:s} in the data specification executor has not yet "
            "been implemented".format(command))


class UnimplementedDSGCommand(DataSpecificationException):
    """ An exception which occurs when trying to write an unimplemented\
        command
    """

    def __init__(self, command):
        """

        :param command: Command attempted to be generated by the DSG
        :type command: str
        """
        Exception.__init__(
            self,
            "Command {0:s} in the data specification generator has not yet "
            "been implemented".format(command))
