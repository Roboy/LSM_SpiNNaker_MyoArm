"""Command-line administrative job management interface.

``spalloc-job`` may be called with a job ID, or if no arguments supplied your
currently running job is shown by default. Various actions may be taken and
each is described below.

Displaying job information
--------------------------

By default, the command displays all known information about a job.

The ``--watch`` option may be added which will cause the output to be updated
in real-time as a job's state changes. For example::

    $ spalloc-job --watch

.. image:: _static/spalloc_job.gif
    :alt: spalloc-job displaying job information.

Controlling board power
-----------------------

The boards allocated to a job may be reset or powered on/off on demand (by
anybody, at any time) by adding the ``--power-on``, ``--power-off`` or
``--reset`` options. For example::

    $ spalloc-job --reset

.. note::

    This command blocks until the action is completed.

Listing board IP addresses
--------------------------

The hostnames of Ethernet-attached chips can be listed in CSV format by adding
the --ethernet-ips argument::

    $ spalloc-job --ethernet-ips
    x,y,hostname
    0,0,192.168.1.97
    0,12,192.168.1.105
    4,8,192.168.1.129
    4,20,192.168.1.137
    8,4,192.168.1.161
    8,16,192.168.1.169

Destroying/Cancelling Jobs
--------------------------

Jobs can be destroyed (by anybody, at any time) using the ``--destroy`` option
which optionally accepts a human-readable explanation::

    $ spalloc-job --destroy "Your job is taking too long..."

.. warning::

    That this "super power" should be used carefully since the user may not be
    notified that their job was destroyed and the first sign of this will be
    their boards being powered down and re-partitioned ready for another user.
"""
import sys
import argparse
import datetime

from pytz import utc
from tzlocal import get_localzone

from collections import OrderedDict

from six import iteritems

from spalloc import config
from spalloc import \
    __version__, ProtocolClient, ProtocolTimeoutError, JobState
from spalloc.term import \
    Terminal, render_definitions, render_boards, DEFAULT_BOARD_EDGES


# The acceptable range of server version numbers
VERSION_RANGE_START = (0, 1, 0)
VERSION_RANGE_STOP = (2, 0, 0)


def show_job_info(t, client, timeout, job_id):
    """Print a human-readable overview of a Job's attributes.

    Parameters
    ----------
    t : :py:class:`.Terminal`
        An output styling object for stdout.
    client : :py:class:`.ProtocolClient`
        A connection to the server.
    timeout : float or None
        The timeout for server responses.
    job_id : int
        The job ID of interest.

    Returns
    -------
    int
        An error code, 0 for success.
    """
    # Get the complete job information (if the job is alive)
    job_list = client.list_jobs(timeout=timeout)
    job = [job for job in job_list if job["job_id"] == job_id]

    if not job:
        # Job no longer exists, just print basic info
        job = client.get_job_state(job_id, timeout=timeout)

        info = OrderedDict()
        info["Job ID"] = job_id
        info["State"] = JobState(job["state"]).name
        if job["reason"] is not None:
            info["Reason"] = job["reason"]
        print(render_definitions(info))
    else:
        # Job is enqueued, show all info
        machine_info = client.get_job_machine_info(job_id, timeout=timeout)
        job = job[0]

        info = OrderedDict()
        info["Job ID"] = job_id
        info["Owner"] = job["owner"]
        info["State"] = JobState(job["state"]).name
        if job["start_time"] is not None:
            utc_timestamp = datetime.datetime.fromtimestamp(
                job["start_time"], utc)
            local_timestamp = utc_timestamp.astimezone(get_localzone())
            info["Start time"] = local_timestamp.strftime('%d/%m/%Y %H:%M:%S')
        info["Keepalive"] = job["keepalive"]

        args = job["args"]
        kwargs = job["kwargs"]
        info["Request"] = "Job({}{}{})".format(
            ", ".join(map(str, args)),
            ",\n    " if args and kwargs else "",
            ",\n    ".join("{}={!r}".format(k, v) for
                           k, v in sorted(iteritems(kwargs)))
        )

        if job["boards"] is not None:
            info["Allocation"] = render_boards([(
                job["boards"],
                t.dim(" . "),
                tuple(map(t.dim, DEFAULT_BOARD_EDGES)),
                tuple(map(t.bright, DEFAULT_BOARD_EDGES)),
            )])

        if machine_info["connections"] is not None:
            connections = sorted(machine_info["connections"])
            info["Hostname"] = connections[0][1]
        if machine_info["width"] is not None:
            info["Width"] = machine_info["width"]
        if machine_info["height"] is not None:
            info["Height"] = machine_info["height"]
        if job["boards"] is not None:
            info["Num boards"] = len(job["boards"])
        if job["power"] is not None:
            info["Board power"] = "on" if job["power"] else "off"
        if job["allocated_machine_name"] is not None:
            info["Running on"] = job["allocated_machine_name"]

        print(render_definitions(info))

    return 0


def watch_job(t, client, timeout, job_id):
    """Re-print a job's information whenever the job changes.

    Parameters
    ----------
    t : :py:class:`.Terminal`
        An output styling object for stdout.
    client : :py:class:`.ProtocolClient`
        A connection to the server.
    timeout : float or None
        The timeout for server responses.
    job_id : int
        The job ID of interest.

    Returns
    -------
    int
        An error code, 0 for success.
    """
    client.notify_job(job_id, timeout=timeout)
    while True:
        t.stream.write(t.clear_screen())
        show_job_info(t, client, timeout, job_id)

        try:
            client.wait_for_notification()
            print("")
        except KeyboardInterrupt:
            # Gracefully exit
            print("")
            break

    return 0


def power_job(client, timeout, job_id, power):
    """Power a job's boards on/off and wait for the action to complete.

    Parameters
    ----------
    client : :py:class:`.ProtocolClient`
        A connection to the server.
    timeout : float or None
        The timeout for server responses.
    job_id : int
        The job ID of interest.
    power : bool
        True = turn on/reset, False = turn off.

    Returns
    -------
    int
        An error code, 0 for success. Fails if the job is not allocated any
        boards or if waiting for the state change notification is interrupted
        by the user.
    """
    if power:
        client.power_on_job_boards(job_id, timeout=timeout)
    else:
        client.power_off_job_boards(job_id, timeout=timeout)

    # Wait for power command to complete...
    while True:
        client.notify_job(job_id, timeout=timeout)
        state = client.get_job_state(job_id, timeout=timeout)

        if state["state"] == JobState.ready:
            # Power command completed
            return 0
        elif state["state"] == JobState.power:
            # Wait for change...
            try:
                client.wait_for_notification()
            except KeyboardInterrupt:
                # If interrupted, quietly return an error state
                return 7
        else:
            # In an unknown state, perhaps the job was queued etc.
            sys.stderr.write(
                "Error: Cannot power {} job {} in state {}.\n".format(
                    "on" if power else "off",
                    job_id,
                    JobState(state["state"]).name))
            return 8


def list_ips(client, timeout, job_id):
    """Print a CSV of board hostnames for all boards allocated to a job.

    Parameters
    ----------
    client : :py:class:`.ProtocolClient`
        A connection to the server.
    timeout : float or None
        The timeout for server responses.
    job_id : int
        The job ID of interest.

    Returns
    -------
    int
        An error code, 0 for success. Fails if the job is not allocated any
        boards.
    """
    info = client.get_job_machine_info(job_id, timeout=timeout)
    connections = info["connections"]
    if connections is not None:
        print("x,y,hostname")
        for ((x, y), hostname) in sorted(connections):
            print("{},{},{}".format(x, y, hostname))
        return 0
    else:
        sys.stderr.write(
            "Job {} is queued or does not exist.\n".format(
                job_id))
        return 9


def destroy_job(client, timeout, job_id, reason=None):
    """Destroy a running job.

    Parameters
    ----------
    client : :py:class:`.ProtocolClient`
        A connection to the server.
    timeout : float or None
        The timeout for server responses.
    job_id : int
        The job ID of interest.
    reason : str or None
        The human-readable reason for destroying the job.

    Returns
    -------
    int
        An error code, 0 for success.
    """
    client.destroy_job(job_id, reason, timeout=timeout)
    return 0


def main(argv=None):
    t = Terminal()

    cfg = config.read_config()

    parser = argparse.ArgumentParser(
        description="Manage running jobs.")

    parser.add_argument("--version", "-V", action="version",
                        version=__version__)

    parser.add_argument("job_id", type=int, nargs="?",
                        help="the job ID of interest, optional if the current "
                             "owner only has one job")

    parser.add_argument("--owner", "-o", default=cfg["owner"],
                        help="if no job ID is provided and this owner has "
                             "only one job, this job is assumed "
                             "(default: %(default)s)")

    control_args = parser.add_mutually_exclusive_group()

    control_args.add_argument("--info", "-i", action="store_true",
                              help="Show basic job information (the default)")
    control_args.add_argument("--watch", "-w", action="store_true",
                              help="watch this job for state changes")
    control_args.add_argument("--power-on", "--reset", "-p", "-r",
                              action="store_true",
                              help="power-on or reset the job's boards")
    control_args.add_argument("--power-off", action="store_true",
                              help="power-off the job's boards")
    control_args.add_argument("--ethernet-ips", "-e", action="store_true",
                              help="output the IPs of all Ethernet connected "
                                   "chips as a CSV")
    control_args.add_argument("--destroy", "-D", nargs="?", metavar="REASON",
                              const="",
                              help="destroy a queued or running job")

    server_args = parser.add_argument_group("spalloc server arguments")

    server_args.add_argument("--hostname", "-H", default=cfg["hostname"],
                             help="hostname or IP of the spalloc server "
                                  "(default: %(default)s)")
    server_args.add_argument("--port", "-P", default=cfg["port"],
                             type=int,
                             help="port number of the spalloc server "
                                  "(default: %(default)s)")
    server_args.add_argument("--timeout", default=cfg["timeout"],
                             type=float, metavar="SECONDS",
                             help="seconds to wait for a response "
                                  "from the server (default: %(default)s)")

    args = parser.parse_args(argv)

    # Fail if server not specified
    if args.hostname is None:
        parser.error("--hostname of spalloc server must be specified")

    # Fail if job *and* owner not specified
    if args.job_id is None and args.owner is None:
        parser.error("job ID (or --owner) not specified")

    client = ProtocolClient(args.hostname, args.port)
    try:
        # Connect to server and ensure compatible version
        client.connect()
        version = tuple(
            map(int, client.version(timeout=args.timeout).split(".")))
        if not (VERSION_RANGE_START <= version < VERSION_RANGE_STOP):
            sys.stderr.write("Incompatible server version ({}).\n".format(
                ".".join(map(str, version))))
            return 2

        # If no Job ID specified, attempt to discover one
        if args.job_id is None:
            jobs = client.list_jobs(timeout=args.timeout)
            job_ids = [job["job_id"] for job in jobs
                       if job["owner"] == args.owner]
            if len(job_ids) == 0:
                sys.stderr.write(
                    "Owner {} has no live jobs.\n".format(args.owner))
                return 3
            elif len(job_ids) > 1:
                sys.stderr.write("Ambiguous: {} has {} live jobs: {}\n".format(
                    args.owner, len(job_ids), ", ".join(map(str, job_ids))))
                return 3
            else:
                args.job_id = job_ids[0]

        # Do as the user asked
        if args.watch:
            return watch_job(t, client, args.timeout, args.job_id)
        elif args.power_on:
            return power_job(client, args.timeout, args.job_id, True)
        elif args.power_off:
            return power_job(client, args.timeout, args.job_id, False)
        elif args.ethernet_ips:
            return list_ips(client, args.timeout, args.job_id)
        elif args.destroy is not None:
            # Set default destruction message
            if args.destroy == "" and args.owner:
                args.destroy = "Destroyed by {}".format(args.owner)
            return destroy_job(client, args.timeout, args.job_id, args.destroy)
        else:
            return show_job_info(t, client, args.timeout, args.job_id)

    except (IOError, OSError, ProtocolTimeoutError) as e:
        sys.stderr.write("Error communicating with server: {}\n".format(e))
        return 1
    finally:
        client.close()


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
