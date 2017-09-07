from data_specification import constants, exceptions, memory_region_collection
import struct
from data_specification.memory_region import MemoryRegion


class DataSpecificationExecutorFunctions(object):
    """ This class includes the function related to each of the commands\
        of the data specification file.
    """

    __slots__ = [

        # ????????????
        "spec_reader",

        # ????????????
        "mem_writer",

        # ????????????
        "memory_space",

        # ????????????
        "space_allocated",

        # ????????????
        "current_region",

        # ????????????
        "registers",

        # ????????????
        "mem_regions",

        # ????????????
        "struct_slots",

        # ????????????
        "_cmd_size",

        # ????????????
        "opcode",

        # ????????????
        "use_dest_reg",

        # ????????????
        "use_src1_reg",

        # ????????????
        "use_src2_reg",

        # ????????????
        "dest_reg",

        # ????????????
        "src1_reg",

        # ????????????
        "src2_reg",

        # ????????????
        "data_len"
    ]

    def __init__(self, spec_reader, mem_writer, memory_space):
        """

        :param spec_reader: The object to read the specification language file\
            from
        :type spec_reader:\
            :py:class:`data_specification.abstract_data_reader.\
            AbstractDataReader`
        :param mem_writer: The object to write the memory image to
        :type mem_writer:\
            :py:class:`data_specification.abstract_data_writer.\
            AbstractDataWriter`
        :param space_available: Memory space available for the data to be\
            generated
        :type space_available: int
        """
        self.spec_reader = spec_reader
        self.mem_writer = mem_writer
        self.memory_space = memory_space

        self.space_allocated = 0
        self.current_region = None

        self.registers = [0] * constants.MAX_REGISTERS
        self.mem_regions = memory_region_collection.MemoryRegionCollection(
            constants.MAX_MEM_REGIONS)
        self.struct_slots = [0] * constants.MAX_STRUCT_SLOTS

        # storage objects
        self._cmd_size = None
        self.opcode = None
        self.use_dest_reg = None
        self.use_src1_reg = None
        self.use_src2_reg = None
        self.dest_reg = None
        self.src1_reg = None
        self.src2_reg = None
        self.data_len = None

    def __unpack_cmd__(self, cmd):
        """ Routine to unpack the command read from the data spec file. The\
        parameters of the command are stored in the class data

        :param cmd: The command read form the data spec file
        :type cmd: int
        :return: No value returned
        :rtype: None
        :raise None
        """
        self._cmd_size = (cmd >> 28) & 0x3
        self.opcode = (cmd >> 20) & 0xFF
        self.use_dest_reg = (cmd >> 18) & 0x1 == 0x1
        self.use_src1_reg = (cmd >> 17) & 0x1 == 0x1
        self.use_src2_reg = (cmd >> 16) & 0x1 == 0x1
        self.dest_reg = (cmd >> 12) & 0xF
        self.src1_reg = (cmd >> 8) & 0xF
        self.src2_reg = (cmd >> 4) & 0xF
        self.data_len = (cmd >> 12) & 0x3

    def execute_break(self, cmd):
        """
        This command raises an exception to stop the execution of the data spec
         executor (DSE)

        :param cmd: the command which triggered the function call
        :type cmd: int
        :return: No value returned
        :rtype: None
        :raise data_specification.exceptions.ExecuteBreakInstruction:\
            Raises the exception to break the execution of the DSE
        """
        raise exceptions.ExecuteBreakInstruction(
            self.spec_reader.tell(), self.spec_reader.filename)

    def execute_nop(self, cmd):
        """
        This command executes no operation

        :param cmd: the command which triggered the function call
        :type cmd: int
        :return: No value returned
        :rtype: None
        :raise None
        """
        pass

    def execute_reserve(self, cmd):
        """
        This command reserves a region and assigns some memory space to it

        :param cmd: the command which triggered the function call
        :type cmd: int
        :return: No value returned
        :rtype: None
        :raise data_specification.exceptions.DataSpecificationSyntaxError:\
            If there is an error in the command syntax
        :raise data_specification.exceptions.\
            DataSpecificationParameterOutOfBoundsException:\ If the requested \
            size of the region is beyond the available memory space
        """
        self.__unpack_cmd__(cmd)
        region = cmd & 0x1F  # cmd[4:0]

        if self._cmd_size != constants.LEN2:
            raise exceptions.DataSpecificationSyntaxError(
                "Command {0:s} requires one word as argument (total 2 words), "
                "but the current encoding ({1:X}) is specified to be {2:d} "
                "words long".format(
                    "RESERVE", cmd, self._cmd_size))

        unfilled = (cmd >> 7) & 0x1 == 0x1

        if not self.mem_regions.is_empty(region):
            raise exceptions.DataSpecificationRegionInUseException(region)

        size_encoded = self.spec_reader.read(4)
        size = struct.unpack("<I", str(size_encoded))[0]
        if size & 0x3 != 0:
            size = (size + 4) - (size & 0x3)

        if (size <= 0) or (size > self.memory_space):
            raise exceptions.DataSpecificationParameterOutOfBoundsException(
                "region size", size, 1, self.memory_space, "RESERVE")

        self.mem_regions[region] = MemoryRegion(
            memory_pointer=0, unfilled=unfilled, size=size)
        self.space_allocated += size

    def execute_free(self, cmd):
        raise exceptions.UnimplementedDSECommand("FREE")

    def execute_declare_rng(self, cmd):
        raise exceptions.UnimplementedDSECommand("DECLARE_RNG")

    def execute_random_dist(self, cmd):
        raise exceptions.UnimplementedDSECommand("DECLARE_RANDOM_DIST")

    def execute_get_random_rumber(self, cmd):
        raise exceptions.UnimplementedDSECommand("GET_RANDOM_NUMBER")

    def execute_start_struct(self, cmd):
        raise exceptions.UnimplementedDSECommand("START_STRUCT")

    def execute_struct_elem(self, cmd):
        raise exceptions.UnimplementedDSECommand("STRUCT_ELEM")

    def execute_end_struct(self, cmd):
        raise exceptions.UnimplementedDSECommand("END_STRUCT")

    def execute_start_constructor(self, cmd):
        raise exceptions.UnimplementedDSECommand("START_CONSTRUCTOR")

    def execute_end_constructor(self, cmd):
        raise exceptions.UnimplementedDSECommand("END_CONSTRUCTOR")

    def execute_construct(self, cmd):
        raise exceptions.UnimplementedDSECommand("CONSTRUCT")

    def execute_read(self, cmd):
        raise exceptions.UnimplementedDSECommand("READ")

    def execute_write(self, cmd):
        """
        This command writes the given value in the specified region a number\
         of times as identified by either a value in the command or a register\
         value

        :param cmd: the command which triggered the function call
        :type cmd: int
        :return: No value returned
        :rtype: None
        :raise data_specification.exceptions.DataSpecificationSyntaxError:\
            If there is an error in the command syntax
        """
        self.__unpack_cmd__(cmd)

        if self.use_src2_reg:
            n_repeats = self.registers[self.src2_reg]
        else:
            n_repeats = cmd & 0xFF

        # Convert data length to bytes
        data_len = (1 << self.data_len)

        if self.use_src1_reg:
            value = self.registers[self.src1_reg]
        else:
            if self._cmd_size == constants.LEN2 and data_len != 8:
                read_data = self.spec_reader.read(4)
                value = struct.unpack("<I", str(read_data))[0]
            elif self._cmd_size == constants.LEN3 and data_len == 8:
                read_data = self.spec_reader.read(8)
                value = struct.unpack("<Q", str(read_data))[0]
            else:
                raise exceptions.DataSpecificationSyntaxError(
                    "Command {0:s} requires a value as an argument, but the "
                    "current encoding ({1:X}) is specified to be {2:d} words "
                    "long and the data length command argument is specified "
                    "to be {3:d} bytes long".format(
                        "WRITE", cmd, self._cmd_size,
                        data_len))

        # Perform the writes
        self._write_to_mem(
            value, data_len, n_repeats, "WRITE")

    def execute_write_array(self, cmd):
        """ This command writes an array of values in the specified region

        :param cmd: the command which triggered the function call
        :type cmd: int
        :return: No value returned
        :rtype: None
        :raise None
        """
        length_encoded = self.spec_reader.read(4)
        length = struct.unpack("<I", str(length_encoded))[0]
        value_encoded = self.spec_reader.read(4 * length)
        self._write_bytes_to_mem(value_encoded, "WRITE_ARRAY")

    def execute_write_struct(self, cmd):
        raise exceptions.UnimplementedDSECommand("WRITE_STRUCT")

    def execute_block_copy(self, cmd):
        raise exceptions.UnimplementedDSECommand("BLOCK_COPY")

    def execute_switch_focus(self, cmd):
        """ This command switches the focus to the desired, already allocated,\
            memory region

        :param cmd: the command which triggered the function call
        :type cmd: int
        :return: No value returned
        :rtype: None
        :raise data_specification.exceptions.\
            DataSpecificationRegionUnfilledException: If the focus is being \
            switched to a region of memory which has been declared to be kept \
            unfilled
        """
        self.__unpack_cmd__(cmd)

        if not self.use_src1_reg:
            region = (cmd >> 8) & 0xF
        else:
            region = self.registers[self.src1_reg]

        if self.mem_regions.is_empty(region):
            raise exceptions.DataSpecificationRegionUnfilledException(
                region, "SWITCH_FOCUS")
        else:
            self.current_region = region

    def execute_loop(self, cmd):
        raise exceptions.UnimplementedDSECommand("LOOP")

    def execute_break_loop(self, cmd):
        raise exceptions.UnimplementedDSECommand("BREAK_LOOP")

    def execute_end_loop(self, cmd):
        raise exceptions.UnimplementedDSECommand("END_LOOP")

    def execute_if(self, cmd):
        raise exceptions.UnimplementedDSECommand("IF")

    def execute_else(self, cmd):
        raise exceptions.UnimplementedDSECommand("ELSE")

    def execute_end_if(self, cmd):
        raise exceptions.UnimplementedDSECommand("ENDIF")

    def execute_mv(self, cmd):
        """ This command moves an immediate value to a register or copies the \
            value of a register to another register

        :param cmd: the command which triggered the function call
        :type cmd: int
        :return: No value returned
        :rtype: None
        :raise data_specification.exceptions.DataSpecificationSyntaxError: \
                if the destination register is not correctly specified - the \
                destination must be a register and the appropriate bit needs \
                to be set in the specification
        """
        self.__unpack_cmd__(cmd)

        if not self.use_dest_reg:
            raise exceptions.DataSpecificationSyntaxError(
                "Destination register not correctly specified")

        if self.use_src1_reg:
            self.registers[self.dest_reg] = self.registers[self.src1_reg]
        else:
            data_encoded = self.spec_reader.read(4)
            data = struct.unpack("<I", str(data_encoded))[0]
            self.registers[self.dest_reg] = data

    def execute_get_wr_ptr(self, cmd):
        raise exceptions.UnimplementedDSECommand("GET_WR_PTR")

    def execute_set_wr_ptr(self, cmd):
        address = None
        self.__unpack_cmd__(cmd)
        if self.use_src1_reg == 1:

            # the data is a register
            future_address = self.registers[self.dest_reg]
        else:

            # the data is a raw address
            data_encoded = self.spec_reader.read(4)
            future_address = struct.unpack("<I", str(data_encoded))[0]

        # check that the address is relative or absolute
        if cmd & 0x1 == 1:

            # relative to its current write pointer
            if self.mem_regions[self.current_region] is None:

                raise exceptions.DataSpecificationNoRegionSelectedException(
                    "the write pointer for this region is currently undefined")
            else:

                # relative to the base address of the region (obsolete)
                # noinspection PyTypeChecker
                address = (
                    self.mem_regions[self.current_region].write_pointer +
                    future_address)
        else:
            address = future_address

        # update write pointer
        self.mem_regions[self.current_region].write_pointer = address

    def execute_reset_wr_ptr(self, cmd):
        raise exceptions.UnimplementedDSECommand("RESET_RW_PTR")

    def execute_align_wr_ptr(self, cmd):
        raise exceptions.UnimplementedDSECommand("ALIGN_WR_PTR")

    def execute_arith_op(self, cmd):
        raise exceptions.UnimplementedDSECommand("ARITH_OP")

    def execute_logic_op(self, cmd):
        raise exceptions.UnimplementedDSECommand("LOGIC_OP")

    def execute_reformat(self, cmd):
        raise exceptions.UnimplementedDSECommand("REFORMAT")

    def execute_copy_struct(self, cmd):
        raise exceptions.UnimplementedDSECommand("COPY_STRUCT")

    def execute_copy_param(self, cmd):
        raise exceptions.UnimplementedDSECommand("COPY_PARAM")

    def execute_write_param(self, cmd):
        raise exceptions.UnimplementedDSECommand("WRITE_PARAM")

    def execute_write_param_component(self, cmd):
        raise exceptions.UnimplementedDSECommand("WRITE_PARAM_COMPONENT")

    def execute_print_val(self, cmd):
        raise exceptions.UnimplementedDSECommand("PRINT_VAL")

    def execute_print_txt(self, cmd):
        raise exceptions.UnimplementedDSECommand("PRINT_TXT")

    def execute_print_struct(self, cmd):
        raise exceptions.UnimplementedDSECommand("PRINT_STRUCT")

    def execute_read_param(self, cmd):
        raise exceptions.UnimplementedDSECommand("READ_PARAM")

    def execute_end_spec(self, cmd):
        """ Return the value which terminates the data spec executor

        :param cmd: the command which triggered the function call
        :type cmd: int
        :return: constants.END_SPEC_EXECUTOR
        :rtype: int
        :raise None
        """
        read_data = self.spec_reader.read(4)
        value = struct.unpack("<i", str(read_data))[0]
        if value != -1:
            raise exceptions.DataSpecificationSyntaxError(
                "Command END_SPEC requires an argument equal to -1. The "
                "current argument value is {0:d}".format(value))
        return constants.END_SPEC_EXECUTOR

    def _write_to_mem(self, value, n_bytes, repeat, command):
        """ Write the specified value to data memory the specified amount of\
            times.

            The selected memory region needs to be already allocated

        :param value: the value to be written in the data memory region
        :type value: int
        :param n_bytes: number of bytes that represent the value
        :type n_bytes: int
        :param repeat: the number of times the value is to be repeated
        :type repeat: int
        :param command: the command which is being executed
        :type command: str
        :return: No value returned
        :rtype: None
        :raise data_specification.exceptions.\
            DataSpecificationNoRegionSelectedException: raised if there is no \
            memory region selected for the write operation
        :raise data_specification.exceptions.\
            DataSpecificationRegionNotAllocated: raised if the selected region\
            has not been allocated memory space
        :raise data_specification.exceptions.DataSpecificationNoMoreException:\
            raised if the selected region has not enough available memory to \
            store the required data
        :raise data_specification.exceptions.\
            DataSpecificationUnknownTypeLengthException: raised if the data \
            type size is not 1, 2, 4, or 8 bytes
        """

        if self.current_region is None:
            raise exceptions.DataSpecificationNoRegionSelectedException(
                command)

        if self.mem_regions.is_empty(self.current_region) is None:
            raise exceptions.DataSpecificationRegionNotAllocated(
                self.current_region, command)

        space_allocated = self.mem_regions[self.current_region].allocated_size
        space_used = self.mem_regions[self.current_region].write_pointer

        # noinspection PyTypeChecker
        space_available = space_allocated - space_used
        space_required = n_bytes * repeat

        if space_available < space_required:
            raise exceptions.DataSpecificationNoMoreException(
                space_available, space_required, self.current_region)

        if n_bytes == 1:
            encoded_value = struct.pack("<B", value)
        elif n_bytes == 2:
            encoded_value = struct.pack("<H", value)
        elif n_bytes == 4:
            encoded_value = struct.pack("<I", value)
        elif n_bytes == 8:
            encoded_value = struct.pack("<Q", value)
        else:
            raise exceptions.DataSpecificationUnknownTypeLengthException(
                n_bytes, command)

        encoded_array = encoded_value * repeat
        current_write_ptr = self.mem_regions[self.current_region].write_pointer

        # noinspection PyTypeChecker
        self.mem_regions[self.current_region].region_data[
            current_write_ptr:current_write_ptr + len(
                encoded_array)] = encoded_array
        self.mem_regions[self.current_region].increment_write_pointer(
            len(encoded_array))

    def _write_bytes_to_mem(self, data, command):
        """ Write raw bytes to data memory

            The selected memory region needs to be already allocated

        :param data: the value to be written in the data memory region
        :type value: bytestring
        :param command: the command which is being executed
        :type command: str
        :return: No value returned
        :rtype: None
        :raise data_specification.exceptions.\
            DataSpecificationNoRegionSelectedException: raised if there is no \
            memory region selected for the write operation
        :raise data_specification.exceptions.\
            DataSpecificationRegionNotAllocated: raised if the selected region\
            has not been allocated memory space
        :raise data_specification.exceptions.DataSpecificationNoMoreException:\
            raised if the selected region has not enough available memory to \
            store the required data
        """
        data_length = len(data)

        if self.current_region is None:
            raise exceptions.DataSpecificationNoRegionSelectedException(
                command)

        if self.mem_regions.is_empty(self.current_region) is None:
            raise exceptions.DataSpecificationRegionNotAllocated(
                self.current_region, command)

        space_allocated = self.mem_regions[self.current_region].allocated_size
        space_used = self.mem_regions[self.current_region].write_pointer

        # noinspection PyTypeChecker
        space_available = space_allocated - space_used
        space_required = data_length

        if space_available < space_required:
            raise exceptions.DataSpecificationNoMoreException(
                space_available, space_required, self.current_region)

        current_write_ptr = self.mem_regions[self.current_region].write_pointer

        # noinspection PyTypeChecker
        self.mem_regions[self.current_region].region_data[
            current_write_ptr:current_write_ptr + data_length] = data
        self.mem_regions[self.current_region].increment_write_pointer(
            data_length)
