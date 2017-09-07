from spalloc.version import __version__

# Alias useful objects
from spalloc.protocol_client import ProtocolClient, ProtocolTimeoutError
from spalloc.job import Job, JobDestroyedError, StateChangeTimeoutError
from spalloc.states import JobState
