""" Used to generate memory images from a set of instructions.

    The main part of this package is the\ 
    :py:class:`data_specification.data_specification_generator.DataSpecificationGenerator`\
    class. This is used to generate a "Data Specification", which can then be\
    executed to produce a memory image.  This package also handles this function\
    if required, through the\
    :py:class:`data_specification.data_specification_executor.DataSpecificationExecutor`\
    class.
    
    Functional Requirements
    =======================
    
        * Creation of a Data Specification Language file which can be executed\
          to produce a memory image.
          
            * Any errors that can be checked during the creation of the\
              specification should throw an exception.
              
            * It will be impossible to detect all errors at creation time.
            
            * There should be no assumption of where the data specification is\
              be stored, although a default provision of a way to write the\
              specification to a file is acceptable.
        
        * Execution of a Data Specification Language file, producing a\
          memory image.
          
            * This should detect any errors during execution and report them,\
              halting the execution.
              
            * There should be no assumption of where the data specification is\
              read from, although a default provision of a way to read the\
              specification from a file is acceptable.
    
    Use Cases
    =========
    
    There are a number of use-cases of this library:
        * :py:class:`~data_specification.data_specification_generator.DataSpecificationGenerator`\
          is used to create a compressed memory image which can be expanded\
          later, to reduce the amount of data that needs to be transferred over\
          a slow connection
        * :py:class:`~data_specification.data_specification_executor.DataSpecificationExecutor`\
          is used to execute a previously generated specification at the\
          receiving end of a slow connection.
"""