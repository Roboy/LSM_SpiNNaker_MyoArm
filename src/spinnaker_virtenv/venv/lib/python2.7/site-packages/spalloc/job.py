"""A high-level Python interface for allocating SpiNNaker boards."""

import threading
import time

from collections import namedtuple

from spalloc.protocol_client import ProtocolClient, ProtocolTimeoutError
from spalloc.config import read_config, SEARCH_PATH
from spalloc.states import JobState

import logging

logger = logging.getLogger(__name__)

# In Python 2, no default handler exists for software which doesn't configure
# its own logging so we must add one ourselves as per
# https://docs.python.org/3.1/library/logging.html#configuring-logging-for-a-library
logger.addHandler(logging.StreamHandler())


VERSION_RANGE_START = (0, 4, 0)
VERSION_RANGE_STOP = (2, 0, 0)


class Job(object):
    """A high-level interface for requesting and managing allocations of
    SpiNNaker boards.

    Constructing a :py:class:`.Job` object connects to a `spalloc-server
    <https://github.com/project-rig/spalloc_server>`_ and requests a number of
    SpiNNaker boards. See the :py:meth:`constructor <.Job.__init__>` for
    details of the types of requests which may be made. The job object may then
    be used to monitor the state of the request, control the boards allocated
    and determine their IP addresses.

    In its simplest form, a :py:class:`.Job` can be used as a context manager
    like so::

        >>> from spalloc import Job
        >>> with Job(6) as j:
        ...     my_boot(j.hostname, j.width, j.height)
        ...     my_application(j.hostname)

    In this example a six-board machine is requested and the ``with`` context
    is entered once the allocation has been made and the allocated boards are
    fully powered on. When control leaves the block, the job is destroyed and
    the boards shut down by the server ready for another job.

    For more fine-grained control, the same functionality is available via
    various methods::

        >>> from spalloc import Job
        >>> j = Job(6)
        >>> j.wait_until_ready()
        >>> my_boot(j.hostname, j.width, j.height)
        >>> my_application(j.hostname)
        >>> j.destroy()

    .. note::

        More complex applications may wish to log the following attributes of
        their job to support later debugging efforts:

        * ``job.id`` -- May be used to query the state of the job and find out
          its fate if cancelled or destroyed. The ``spalloc-job`` command can
          be used to discover the state/fate of the job and
          ``spalloc-where-is`` may be used to find out what boards problem
          chips reside on.
        * ``job.machine_name`` and ``job.boards`` together give a complete
          record of the hardware used by the job. The ``spalloc-where-is``
          command may be used to find out the physical locations of the boards
          used.

    :py:class:`.Job` objects have the following attributes which describe the
    job and its allocated machines:

    Attributes
    ----------
    job.id : int or None
        The job ID allocated by the server to the job.
    job.state : :py:class:`.JobState`
        The current state of the job.
    job.power : bool or None
        If boards have been allocated to the job, are they on (True) or off
        (False). None if no boards are allocated to the job.
    job.reason : str or None
        If the job has been destroyed, gives the reason (which may be None), or
        None if the job has not been destroyed.
    job.hostname : str or None
        The hostname of the SpiNNaker chip at (0, 0), or None if no boards have
        been allocated to the job.
    job.connections : {(x, y): hostname, ...} or None
        The hostnames of all Ethernet-connected SpiNNaker chips, or None if no
        boards have been allocated to the job.
    job.width : int or None
        The width of the SpiNNaker network in chips, or None if no boards have
        been allocated to the job.
    job.height : int or None
        The height of the SpiNNaker network in chips, or None if no boards have
        been allocated to the job.
    job.machine_name : str or None
        The name of the machine the boards are allocated in, or None if not yet
        allocated.
    job.boards : [[x, y, z], ...] or None
        The logical coordinates allocated to the job, or None if not yet
        allocated.
    """

    def __init__(self, *args, **kwargs):
        """Request a SpiNNaker machine.

        A :py:class:`.Job` is constructed in one of the following styles::

            >>> # Any single (SpiNN-5) board
            >>> Job()
            >>> Job(1)

            >>> # Any machine with at least 4 boards
            >>> Job(4)

            >>> # Any 7-or-more board machine with an aspect ratio at least as
            >>> # square as 1:2
            >>> Job(7, min_ratio=0.5)

            >>> # Any 4x5 triad segment of a machine (may or may-not be a
            >>> # torus/full machine)
            >>> Job(4, 5)

            >>> # Any torus-connected (full machine) 4x2 machine
            >>> Job(4, 2, require_torus=True)

            >>> # Board x=3, y=2, z=1 on the machine named "m"
            >>> Job(3, 2, 1, machine="m")

            >>> # Keep using (and keeping-alive) an existing allocation
            >>> Job(resume_job_id=123)

        Once finished with a Job, the :py:meth:`.destroy` (or in unusual
        applications :py:meth:`.Job.close`) method must be called to destroy
        the job, close the connection to the server and terminate the
        background keep-alive thread. Alternatively, a Job may be used as a
        context manager which automatically calls :py:meth:`.destroy` on
        exiting the block::

            >>> with Job() as j:
            ...     # ...for example...
            ...     my_boot(j.hostname, j.width, j.height)
            ...     my_application(j.hostname)

        The following keyword-only parameters below are used both to specify
        the server details as well as the job requirements. Most parameters
        default to the values supplied in the local :py:mod:`~spalloc.config`
        file allowing usage as in the examples above.

        Parameters
        ----------
        hostname : str
            **Required.** The name of the spalloc server to connect to. (Read
            from config file if not specified.)
        port : int
            The port number of the spalloc server to connect to. (Read from
            config file if not specified.)
        reconnect_delay : float
            Number of seconds between attempts to reconnect to the server.
            (Read from config file if not specified.)
        timeout : float or None
            Timeout for waiting for replies from the server. If None, will keep
            trying forever. (Read from config file if not specified.)
        config_filenames : [str, ...]
            If given must be a list of filenames to read configuration options
            from. If not supplied, the default config file locations are
            searched. Set to an empty list to prevent using values from config
            files.

        Other Parameters
        ----------------
        resume_job_id : int or None
            If supplied, rather than creating a new job, take on an existing
            one, keeping it alive as required by the original job. If this
            argument is used, all other requirements are ignored.
        owner : str
            **Required.** The name of the owner of the job. By convention this
            should be your email address. (Read from config file if not
            specified.)
        keepalive : float or None
            The number of seconds after which the server may consider the job
            dead if this client cannot communicate with it. If None, no timeout
            will be used and the job will run until explicitly destroyed. Use
            with extreme caution. (Read from config file if not specified.)
        machine : str or None
            Specify the name of a machine which this job must be executed on.
            If None, the first suitable machine available will be used,
            according to the tags selected below. Must be None when tags are
            given. (Read from config file if not specified.)
        tags : [str, ...] or None
            The set of tags which any machine running this job must have. If
            None is supplied, only machines with the "default" tag will be
            used. If machine is given, this argument must be None.  (Read from
            config file if not specified.)
        min_ratio : float
            The aspect ratio (h/w) which the allocated region must be 'at least
            as square as'. Set to 0.0 for any allowable shape, 1.0 to be
            exactly square etc. Ignored when allocating single boards or
            specific rectangles of triads.
        max_dead_boards : int or None
            The maximum number of broken or unreachable boards to allow in the
            allocated region. If None, any number of dead boards is permitted,
            as long as the board on the bottom-left corner is alive. (Read from
            config file if not specified.)
        max_dead_links : int or None
            The maximum number of broken links allow in the allocated region.
            When require_torus is True this includes wrap-around links,
            otherwise peripheral links are not counted.  If None, any number of
            broken links is allowed. (Read from config file if not specified.).
        require_torus : bool
            If True, only allocate blocks with torus connectivity. In general
            this will only succeed for requests to allocate an entire machine.
            Must be False when allocating boards. (Read from config file if not
            specified.)
        """
        # Read configuration
        config_filenames = kwargs.pop("config_filenames", SEARCH_PATH)
        config = read_config(config_filenames)

        # Get protocol client options
        hostname = kwargs.get("hostname", config["hostname"])
        owner = kwargs.get("owner", config["owner"])
        port = kwargs.get("port", config["port"])
        self._reconnect_delay = kwargs.get("reconnect_delay",
                                           config["reconnect_delay"])
        self._timeout = kwargs.get("timeout", config["timeout"])
        if hostname is None:
            raise ValueError("A hostname must be specified.")

        # Cached responses of _get_state and _get_machine_info
        self._last_state = None
        self._last_machine_info = None

        # Connection to server (and associated lock)
        self._client = ProtocolClient(hostname, port)
        self._client_lock = threading.RLock()

        # Set-up (but don't start) background keepalive thread
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_thread,
            name="job-keepalive-thread")
        self._keepalive_thread.daemon = True

        # Event fired when the background thread should shut-down
        self._stop = threading.Event()

        # Check version compatibility (fail fast if can't communicate with
        # server)
        self._client.connect(timeout=self._timeout)
        self._assert_compatible_version()

        # Resume/create the job
        resume_job_id = kwargs.get("resume_job_id", None)
        if resume_job_id:
            self.id = resume_job_id

            # If the job no longer exists, we can't get the keepalive interval
            # (and there's nothing to keepalive) so just bail out.
            job_state = self._get_state()
            if (job_state.state == JobState.unknown or
                    job_state.state == JobState.destroyed):
                raise JobDestroyedError(
                    "Job {} does not exist: {}{}{}".format(
                        resume_job_id,
                        job_state.state.name,
                        ": " if job_state.reason is not None else "",
                        job_state.reason
                        if job_state.reason is not None else ""))

            # Snag the keepalive interval from the job
            self._keepalive = job_state.keepalive

            logger.info("Spalloc resumed job %d", self.id)
        else:
            # Get job creation arguments
            job_args = args
            job_kwargs = {
                "owner": owner,
                "keepalive": kwargs.get("keepalive", config["keepalive"]),
                "machine": kwargs.get("machine", config["machine"]),
                "tags": kwargs.get("tags", config["tags"]),
                "min_ratio": kwargs.get("min_ratio", config["min_ratio"]),
                "max_dead_boards":
                    kwargs.get("max_dead_boards", config["max_dead_boards"]),
                "max_dead_links":
                    kwargs.get("max_dead_links", config["max_dead_links"]),
                "require_torus":
                    kwargs.get("require_torus", config["require_torus"]),
                "timeout": self._timeout,
            }

            # Sanity check arguments
            if job_kwargs["owner"] is None:
                raise ValueError("An owner must be specified.")
            if ((job_kwargs["tags"] is not None) and
                    (job_kwargs["machine"] is not None)):
                raise ValueError(
                    "Only one of tags and machine may be specified.")

            self._keepalive = job_kwargs["keepalive"]

            # Create the job (failing fast if can't communicate)
            self.id = self._client.create_job(*job_args, **job_kwargs)

            logger.info("Created spalloc job %d", self.id)

        # Start keepalive thread now that everything is up
        self._keepalive_thread.start()

    def __enter__(self):
        """Convenience context manager for common case where a new job is to be
        created and then destroyed once some code has executed.

        Waits for machine to be ready before the context enters and frees the
        allocation when the context exits.

        Example::

            >>> from spalloc import Job
            >>> with Job(6) as j:
            ...     my_boot(j.hostname, j.width, j.height)
            ...     my_application(j.hostname)
        """
        logger.info("Waiting for boards to become ready...")
        try:
            self.wait_until_ready()
            return self
        except:
            self.destroy()
            raise

    def __exit__(self, type=None, value=None, traceback=None):
        self.destroy()

    def _assert_compatible_version(self):
        """Assert that the server version is compatible."""
        v = self._client.version(timeout=self._timeout)
        v_ints = tuple(map(int, v.split(".")[:3]))

        if not (VERSION_RANGE_START <= v_ints < VERSION_RANGE_STOP):
            self._client.close()
            raise ValueError(
                "Server version {} is not compatible with this client.".format(
                    v))

    def _reconnect(self):
        """Reconnect to the server and check version.

        If reconnection fails, the error is reported as a warning but no
        exception is raised.
        """
        try:
            self._client.connect(self._timeout)
            self._assert_compatible_version()
            logger.info("Reconnected to spalloc server successfully.")
        except (IOError, OSError) as e:
            # Connect/version command failed... Leave the socket clearly
            # broken so that we retry again
            logger.warning(
                "Spalloc server is unreachable (%s), will keep trying...", e)
            self._client.close()

    def _keepalive_thread(self):
        """Background keep-alive thread."""
        # Send the keepalive packet twice as often as required
        keepalive = self._keepalive
        if keepalive is not None:
            keepalive /= 2.0
        while not self._stop.wait(keepalive):
            with self._client_lock:
                # Keep trying to send the keep-alive packet, if this fails,
                # keep trying to reconnect until it succeeds.
                while not self._stop.is_set():
                    try:
                        self._client.job_keepalive(
                            self.id, timeout=self._timeout)
                        break
                    except (ProtocolTimeoutError, IOError, OSError):
                        # Something went wrong, reconnect, after a delay which
                        # may be interrupted by the thread being stopped
                        self._client.close()
                        if not self._stop.wait(self._reconnect_delay):
                            self._reconnect()

    def destroy(self, reason=None):
        """Destroy the job and disconnect from the server.

        Parameters
        ----------
        reason : str or None
            *Optional.* Gives a human-readable explanation for the destruction
            of the job.
        """
        # Attempt to inform the server that the job was destroyed, fail
        # quietly on failure since the server will eventually time-out the job
        # itself.
        try:
            self._client.destroy_job(self.id, reason)
        except (IOError, OSError, ProtocolTimeoutError) as e:
            logger.warning("Could not destroy spalloc job: %s", e)

        self.close()

    def close(self):
        """Disconnect from the server and stop keeping the job alive.

        .. warning::

            This method does not free the resources allocated by the job but
            rather simply disconnects from the server and ceases sending
            keep-alive messages. Most applications should use
            :py:meth:`.destroy` instead.
        """
        # Stop background thread
        self._stop.set()
        self._keepalive_thread.join()

        # Disconnect
        self._client.close()

    def _get_state(self):
        """Get the state of the job.

        Returns
        -------
        :py:class:`._JobStateTuple`
        """
        with self._client_lock:
            state = self._client.get_job_state(self.id, timeout=self._timeout)
            return _JobStateTuple(
                state=JobState(state["state"]),
                power=state["power"],
                keepalive=state["keepalive"],
                reason=state["reason"],
            )

    def set_power(self, power):
        """Turn the boards allocated to the job on or off.

        Does nothing if the job has not yet been allocated any boards.

        The :py:meth:`.wait_until_ready` method may be used to wait for the
        boards to fully turn on or off.

        Parameters
        ----------
        power : bool
            True to power on the boards, False to power off. If the boards are
            already turned on, setting power to True will reset them.
        """
        with self._client_lock:
            if power:
                self._client.power_on_job_boards(
                    self.id, timeout=self._timeout)
            else:
                self._client.power_off_job_boards(
                    self.id, timeout=self._timeout)

    def reset(self):
        """Reset (power-cycle) the boards allocated to the job.

        Does nothing if the job has not been allocated.

        The :py:meth:`.wait_until_ready` method may be used to wait for the
        boards to fully turn on or off.
        """
        self.set_power(True)

    def _get_machine_info(self):
        """Get information about the boards allocated to the job, e.g. the IPs
        and system dimensions.

        Returns
        -------
        :py:class:`._JobMachineInfoTuple`
        """
        with self._client_lock:
            info = self._client.get_job_machine_info(
                self.id, timeout=self._timeout)

            return _JobMachineInfoTuple(
                width=info["width"],
                height=info["height"],
                connections=({(x, y): hostname
                              for (x, y), hostname
                              in info["connections"]}
                             if info["connections"] is not None
                             else None),
                machine_name=info["machine_name"],
                boards=info["boards"],
            )

    @property
    def state(self):
        """The current state of the job."""
        self._last_state = self._get_state()
        return self._last_state.state

    @property
    def power(self):
        """Are the boards powered/powering on or off?"""
        self._last_state = self._get_state()
        return self._last_state.power

    @property
    def reason(self):
        """For what reason was the job destroyed (if any and if destroyed)."""
        self._last_state = self._get_state()
        return self._last_state.reason

    @property
    def connections(self):
        """The list of Ethernet connected chips and their IPs.

        Returns
        -------
        {(x, y): hostname, ...} or None
        """
        # Note that the connections for a job will never change once defined so
        # only need to get this once.
        if (self._last_machine_info is None or
                self._last_machine_info.connections is None):
            self._last_machine_info = self._get_machine_info()

        return self._last_machine_info.connections

    @property
    def hostname(self):
        """The hostname of chip 0, 0 (or None if not allocated yet)."""
        return self.connections[(0, 0)]

    @property
    def width(self):
        """The width of the allocated machine in chips (or None)."""
        # Note that the dimensions of a job will never change once defined so
        # only need to get this once.
        if (self._last_machine_info is None or
                self._last_machine_info.width is None):
            self._last_machine_info = self._get_machine_info()

        return self._last_machine_info.width

    @property
    def height(self):
        """The height of the allocated machine in chips (or None)."""
        # Note that the dimensions of a job will never change once defined so
        # only need to get this once.
        if (self._last_machine_info is None or
                self._last_machine_info.height is None):
            self._last_machine_info = self._get_machine_info()

        return self._last_machine_info.height

    @property
    def machine_name(self):
        """The name of the machine the job is allocated on (or None)."""
        # Note that the machine will never change once defined so only need to
        # get this once.
        if (self._last_machine_info is None or
                self._last_machine_info.machine_name is None):
            self._last_machine_info = self._get_machine_info()

        return self._last_machine_info.machine_name

    @property
    def boards(self):
        """The coordinates of the boards allocated for the job (or None)."""
        # Note that the machine will never change once defined so only need to
        # get this once.
        if (self._last_machine_info is None or
                self._last_machine_info.machine_name is None):
            self._last_machine_info = self._get_machine_info()

        return self._last_machine_info.boards

    def wait_for_state_change(self, old_state, timeout=None):
        """Block until the job's state changes from the supplied state.

        Parameters
        ----------
        old_state : :py:class:`~spalloc.JobState`
            The current state.
        timeout : float or None
            The number of seconds to wait for a change before timing out. If
            None, wait forever.

        Returns
        -------
        :py:class:`~spalloc.JobState`
            The new state, or old state if timed out.
        """
        finish_time = time.time() + timeout if timeout is not None else None

        # We may get disconnected while waiting so keep listening...
        while finish_time is None or finish_time > time.time():
            try:
                # Watch for changes in this Job's state
                with self._client_lock:
                    self._client.notify_job(self.id)

                # Wait for job state to change
                while finish_time is None or finish_time > time.time():
                    # Has the job changed state?
                    new_state = self._get_state().state
                    if new_state != old_state:
                        return new_state

                    # Wait for a state change and keep the job alive
                    with self._client_lock:
                        # Since we're about to block holding the client lock,
                        # we must be responsible for keeping everything alive.
                        while finish_time is None or finish_time > time.time():
                            self._client.job_keepalive(
                                self.id, timeout=self._timeout)

                            # Wait for the job to change
                            try:
                                # Block waiting for the job to change no-longer
                                # than the user-specified timeout or half the
                                # keepalive interval.
                                if (finish_time is not None and
                                        self._keepalive is not None):
                                    time_left = finish_time - time.time()
                                    wait_timeout = min(self._keepalive / 2.0,
                                                       time_left)
                                elif finish_time is None:
                                    if self._keepalive is None:
                                        wait_timeout = None
                                    else:
                                        wait_timeout = self._keepalive / 2.0
                                else:
                                    wait_timeout = finish_time - time.time()
                                if wait_timeout is None or wait_timeout >= 0.0:
                                    self._client.wait_for_notification(
                                        wait_timeout)
                                    break
                            except ProtocolTimeoutError:
                                # Its been a while, send a keep-alive since
                                # we're still holding the lock
                                pass
                        else:
                            # The user's timeout expired while waiting for a
                            # state change, return the old state and give up.
                            return old_state
            except (IOError, OSError, ProtocolTimeoutError):
                # Something went wrong while communicating with the server,
                # reconnect after the reconnection delay (or timeout, whichever
                # came first.
                with self._client_lock:
                    self._client.close()
                    if finish_time is not None:
                        delay = min(finish_time - time.time(),
                                    self._reconnect_delay)
                    else:
                        delay = self._reconnect_delay
                    time.sleep(max(0.0, delay))
                    self._reconnect()

        # If we get here, the timeout expired without a state change, just
        # return the old state
        return old_state

    def wait_until_ready(self, timeout=None):
        """Block until the job is allocated and ready.

        Parameters
        ----------
        timeout : float or None
            The number of seconds to wait before timing out. If None, wait
            forever.

        Raises
        ------
        StateChangeTimeoutError
            If the timeout expired before the ready state was entered.
        JobDestroyedError
            If the job was destroyed before becoming ready.
        """
        cur_state = None
        finish_time = time.time() + timeout if timeout is not None else None
        while finish_time is None or finish_time > time.time():
            if cur_state is None:
                # Get initial state (NB: done here such that the command is
                # never sent if the timeout has already occurred)
                cur_state = self._get_state().state

            # Are we ready yet?
            if cur_state == JobState.ready:
                # Now in the ready state!
                return
            elif cur_state == JobState.queued:
                logger.info("Job has been queued by the spalloc server.")
            elif cur_state == JobState.power:
                logger.info("Waiting for board power commands to complete.")
            elif cur_state == JobState.destroyed:
                # In a state which can never become ready
                raise JobDestroyedError(self._get_state().reason)
            elif cur_state == JobState.unknown:
                # Server has forgotten what this job even was...
                raise JobDestroyedError(
                    "Spalloc server no longer recognises job.")

            # Wait for a state change...
            if finish_time is None:
                time_left = None
            else:
                time_left = finish_time - time.time()
            cur_state = self.wait_for_state_change(cur_state, time_left)

        # Timed out!
        raise StateChangeTimeoutError()


class StateChangeTimeoutError(Exception):
    """Thrown when a state change takes too long to occur."""


class JobDestroyedError(Exception):
    """Thrown when the job was destroyed while waiting for it to become
    ready.
    """


class _JobStateTuple(namedtuple("_JobStateTuple",
                                "state,power,keepalive,reason")):
    """Tuple describing the state of a particular job, returned by
    :py:meth:`.Job._get_state`.

    Parameters
    ----------
    state : :py:class:`.JobState`
        The current state of the queried job.
    power : bool or None
        If job is in the ready or power states, indicates whether the boards
        are power{ed,ing} on (True), or power{ed,ing} off (False). In other
        states, this value is None.
    keepalive : float or None
        The Job's keepalive value: the number of seconds between queries
        about the job before it is automatically destroyed. None if no
        timeout is active (or when the job has been destroyed).
    reason : str or None
        If the job has been destroyed, this may be a string describing the
        reason the job was terminated.
    """

    # Python 3.4 Workaround: https://bugs.python.org/issue24931
    __slots__ = tuple()


class _JobMachineInfoTuple(namedtuple("_JobMachineInfoTuple",
                                      "width,height,connections,"
                                      "machine_name,boards")):
    """Tuple describing the machine alloated to a job, returned by
    :py:meth:`.Job._get_machine_info`.

    Parameters

    from collections import namedtuple
    ----------
    width, height : int or None
        The dimensions of the machine in *chips* or None if no machine
        allocated.
    connections : {(x, y): hostname, ...} or None
        A dictionary mapping from SpiNNaker Ethernet-connected chip coordinates
        in the machine to hostname or None if no machine allocated.
    machine_name : str or None
        The name of the machine the job is allocated on or None if no machine
        allocated.
    boards : [[x, y, z], ...] or None
        The logical board coordinates of all boards allocated to the job or
        None if none allocated yet.
    """

    # Python 3.4 Workaround: https://bugs.python.org/issue24931
    __slots__ = tuple()
