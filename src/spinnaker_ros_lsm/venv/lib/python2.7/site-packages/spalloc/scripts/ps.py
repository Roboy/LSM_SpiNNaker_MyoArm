"""An administrative command-line process listing utility.

By default, the ``spalloc-ps`` command lists all running and queued jobs.  For
a real-time monitor of queued and running jobs, the ``--watch`` option may be
added.

.. image:: _static/spalloc_ps.png
    :alt: Jobs being listed by spalloc-ps

This list may be filtered by owner or machine with the ``--owner`` and
``--machine`` arguments.
"""

import sys
import argparse
import datetime

from pytz import utc
from tzlocal import get_localzone

from spalloc import config
from spalloc import \
    __version__, ProtocolClient, ProtocolTimeoutError, JobState
from spalloc.term import Terminal, render_table


# The acceptable range of server version numbers
VERSION_RANGE_START = (0, 1, 0)
VERSION_RANGE_STOP = (2, 0, 0)


def render_job_list(t, jobs, machine=None, owner=None):
    """Return a human-readable process listing.

    Parameters
    ----------
    t : :py:class:`spalloc.term.Terminal`
        The terminal to which the output will be sent.
    jobs : [{...}, ...]
        The list of jobs returned by the server.
    machine : str or None
        If not None, only list jobs on this machine.
    owner : str or None
        If not None, only list jobs with this owner.
    """
    table = []

    # Add headings
    table.append(((t.underscore_bright, "ID"),
                  (t.underscore_bright, "State"),
                  (t.underscore_bright, "Power"),
                  (t.underscore_bright, "Boards"),
                  (t.underscore_bright, "Machine"),
                  (t.underscore_bright, "Created at"),
                  (t.underscore_bright, "Keepalive"),
                  (t.underscore_bright, "Owner")))

    for job in jobs:
        # Filter jobs
        if machine is not None and job["allocated_machine_name"] != machine:
            continue
        if owner is not None and job["owner"] != owner:
            continue

        # Colourise job states
        if job["state"] == JobState.queued:
            job_state = (t.blue, "queue")
        elif job["state"] == JobState.power:
            job_state = (t.yellow, "power")
        elif job["state"] == JobState.ready:
            job_state = (t.green, "ready")
        else:
            job_state = str(job["state"])

        # Colourise power states
        if job["power"] is not None:
            if job["power"]:
                power_state = (t.green, "on")
            else:
                power_state = (t.red, "off")

            if job["state"] == JobState.power:
                power_state = (t.yellow, power_state[1])
        else:
            power_state = ""

        if job["boards"] is not None:
            num_boards = len(job["boards"])
        else:
            num_boards = ""

        # Format start time
        utc_timestamp = datetime.datetime.fromtimestamp(
            job["start_time"], utc)
        local_timestamp = utc_timestamp.astimezone(get_localzone())
        timestamp = local_timestamp.strftime('%d/%m/%Y %H:%M:%S')

        if job["allocated_machine_name"] is not None:
            machine_name = job["allocated_machine_name"]
        else:
            machine_name = ""

        table.append((
            job["job_id"],
            job_state,
            power_state,
            num_boards,
            machine_name,
            timestamp,
            str(job["keepalive"]),
            job["owner"],
        ))

    # Format the table
    return render_table(table)


def main(argv=None):
    t = Terminal(stream=sys.stderr)

    cfg = config.read_config()

    parser = argparse.ArgumentParser(
        description="List all active jobs.")

    parser.add_argument("--version", "-V", action="version",
                        version=__version__)

    parser.add_argument("--watch", "-w", action="store_true", default=False,
                        help="watch the list of live jobs in real time")

    filter_args = parser.add_argument_group("filtering arguments")

    filter_args.add_argument("--machine", "-m",
                             help="list only jobs on the specified "
                                  "machine")
    filter_args.add_argument("--owner", "-o",
                             help="list only jobs belonging to a particular "
                                  "owner")

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

        if args.watch:
            client.notify_job(timeout=args.timeout)

        while True:
            jobs = client.list_jobs(timeout=args.timeout)

            # Clear the screen before reprinting the table
            if args.watch:
                sys.stdout.write(t.clear_screen())

            print(render_job_list(
                t, jobs, args.machine, args.owner))

            # Exit or wait for changes, if requested
            if not args.watch:
                return 0
            else:
                # Wait for state change
                try:
                    client.wait_for_notification()
                except KeyboardInterrupt:
                    # Gracefully exit
                    print("")
                    return 0

                # Print a newline to separate old table from the new table when
                # it gets printed if ANSI screen clearing is not possible.
                print("")

    except (IOError, OSError, ProtocolTimeoutError) as e:
        sys.stderr.write("Error communicating with server: {}\n".format(e))
        return 1
    finally:
        client.close()


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
