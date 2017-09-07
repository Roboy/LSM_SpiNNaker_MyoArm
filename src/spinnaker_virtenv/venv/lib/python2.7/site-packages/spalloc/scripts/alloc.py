"""A command-line utility for creating jobs.

.. note::

    In the examples below, it is assumed that the spalloc server hostname and a
    suitable owner name have been specified in a :py:mod:`~spalloc.config`
    file.


Basic usage
-----------

By default, the ``spalloc`` command allocates a job according to the
command-line specification and then waits for boards to be allocated and
powered on.

.. note::

    By default, allocated machines are powered on but not booted. If Rig_ is
    installed, ``spalloc`` provides a ``--boot`` option which also boots the
    allocated machine once it has been powered on.

.. image:: _static/spalloc.gif
    :alt: Animated GIF showing the typical execution of a spalloc call.

The ``spalloc`` command can be called in one of the following styles though
most users will probably only require the first two.

=====================  ====================================================
Invocation             Allocation
=====================  ====================================================
``spalloc``            A single SpiNN-5 board
``spalloc 5``          A machine with *at least* 5 boards
``spalloc 4 2``        A 4x2 *triad* machine.
``spalloc 3 4 0``      A single SpiNN-5 board at logical position (3, 4, 0)
=====================  ====================================================

.. _Rig: https://github.com/project-rig/rig

A range of additional command-line arguments are available to control various
aspects of Job allocation, run ``spalloc --help`` for a complete listing.


Wrapping other commands
-----------------------

The ``spalloc`` command can alternatively wrap an existing command, calling it
once a SpiNNaker machine is allocated and cleaning up the job when the command
exits::

    $ spalloc 24 -c "rig-boot {} {w} {h} && python my_app.py {}"

The example above attempts to allocate a 24-board machine and, once allocated
and powered on, calls the command above, with the arguments in curly braces
substituted for details of the allocated machine.

The following substitutions are available:

==================  =================================
Token               Substitution
==================  =================================
``{}``              Chip 0, 0's hostname
``{hostname}``      Chip 0, 0's hostname
``{w}``             Width of the system (in chips)
``{width}``         Width of the system (in chips)
``{h}``             Height of the system (in chips)
``{height}``        Height of the system (in chips)
``{ethernet_ips}``  Filename of a CSV of Ethernet IPs
``{id}``            The job ID
==================  =================================

Ethernet-connected chip hostname CSV Format
-------------------------------------------

Hostnames for all Ethernet-connected SpiNNaker chips in a machine are provided
in a CSV file with three columns:: x, y and hostname. The CSV file is newline
(``\\n``) delimited and the first row is a header.

Disconnecting and resuming jobs
-------------------------------

.. warning::

    This functionality is intended for advanced users only.

By default, when the ``spalloc`` command exits, the job will be destroyed and
any allocated boards freed. This behaviour can be disabled with the
``--no-destroy`` argument, leaving the job allocated after the command exits.

Such a job may be 'resumed' by calling ``spalloc`` with the ``--resume
[JOB_ID]`` option.

Note that by default, jobs require a 'keepalive' message to be sent to the
server at a regular interval. While the ``spalloc`` command is running, these
messages are sent automatically but after exiting the commands are no longer
sent. Adding the ``--keepalive -1`` option when creating a job disables this.
"""

import os
import sys
import argparse
import tempfile
import logging
import subprocess

from collections import OrderedDict

from six import iteritems
from six.moves import input, shlex_quote

from spalloc import config
from spalloc import Job, JobState, __version__
from spalloc.term import Terminal, render_definitions


# Rig is used to implement the optional '--boot' facility.
try:
    from rig.machine_control import MachineController
except ImportError:  # pragma: no cover
    MachineController = None


def write_ips_to_csv(connections, ip_file_filename):
    """Write the supplied IP addresses to a CSV file.

    The produced CSV has three columns: x, y and hostname where x and y give
    chip coordinates of Ethernet-connected chips and hostname gives the IP
    address for that chip.

    Parameters
    ----------
    connections : {(x, y): hostname, ...}
    ip_file_filename : str
    """
    with open(ip_file_filename, "w") as f:
        f.write("x,y,hostname\n")
        f.write("".join("{},{},{}\n".format(x, y, hostname)
                        for (x, y), hostname
                        in sorted(iteritems(connections))))


def print_info(machine_name, connections, width, height, ip_file_filename):
    """Print the current machine info in a human-readable form and wait for the
    user to press enter.

    Parameters
    ----------
    machine_name : str
        The machine the job is running on.
    connections : {(x, y): hostname, ...}
        The connections to the boards.
    width, height : int
        The width and height of the machine in chips.
    ip_file_filename : str
    """
    t = Terminal()

    to_print = OrderedDict()

    to_print["Hostname"] = t.bright(connections[(0, 0)])
    to_print["Width"] = width
    to_print["Height"] = height

    if len(connections) > 1:
        to_print["Num boards"] = len(connections)
        to_print["All hostnames"] = ip_file_filename

    to_print["Running on"] = machine_name

    print(render_definitions(to_print))

    try:
        input(t.dim("<Press enter when done>"))
    except (KeyboardInterrupt, EOFError):
        print("")


def run_command(command, job_id, machine_name, connections, width, height,
                ip_file_filename):
    """Run a user-specified command, substituting arguments for values taken
    from the allocated board.

    Arguments may include the following substitution tokens:

    ==================  =================================
    Token               Substitution
    ==================  =================================
    ``{}``              Chip 0, 0's hostname
    ``{hostname}``      Chip 0, 0's hostname
    ``{w}``             Width of the system (in chips)
    ``{width}``         Width of the system (in chips)
    ``{h}``             Height of the system (in chips)
    ``{height}``        Height of the system (in chips)
    ``{ethernet_ips}``  Filename of a CSV of Ethernet IPs
    ``{id}``            The job ID
    ==================  =================================

    Parameters
    ----------
    command : [command, arg, ...]
    machine_name : str
        The machine the job is running on.
    connections : {(x, y): hostname, ...}
        The connections to the boards.
    width, height : int
        The width and height of the machine in chips.
    ip_file_filename : str

    Returns
    -------
    int
        The return code of the supplied command.
    """

    root_hostname = connections[(0, 0)]

    # Print essential info in log
    logging.info("Allocated %d x %d chip machine in '%s'",
                 width, height, machine_name)
    logging.info("Chip (0, 0) IP: %s", root_hostname)
    logging.info("All board IPs listed in: %s", ip_file_filename)

    # Make substitutions in command arguments
    command = [arg.format(root_hostname,
                          hostname=root_hostname,
                          w=width,
                          width=width,
                          h=height,
                          height=height,
                          ethernet_ips=ip_file_filename,
                          id=job_id)
               for arg in command]

    # NB: When using shell=True, commands should be given as a string rather
    # than the usual list of arguments.
    command = " ".join(map(shlex_quote, command))
    p = subprocess.Popen(command, shell=True)

    # Pass through keyboard interrupts
    while True:
        try:
            return p.wait()
        except KeyboardInterrupt:
            p.terminate()


def main(argv=None):
    t = Terminal(stream=sys.stderr)

    cfg = config.read_config()

    parser = argparse.ArgumentParser(
        description="Request (and allocate) a SpiNNaker machine.")

    parser.add_argument("--version", "-V", action="version",
                        version=__version__)

    parser.add_argument("--quiet", "-q", action="store_true",
                        default=False,
                        help="suppress informational messages")
    parser.add_argument("--debug", action="store_true",
                        default=False,
                        help="enable additional diagnostic information")
    parser.add_argument("--no-destroy", "-D", action="store_true",
                        default=False,
                        help="do not destroy the job on exit")

    if MachineController is not None:
        parser.add_argument("--boot", "-B", action="store_true",
                            default=False,
                            help="boot the machine once powered on")

    allocation_args = parser.add_argument_group(
        "allocation requirement arguments")
    allocation_args.add_argument("what", nargs="*", default=[], type=int,
                                 metavar="WHAT",
                                 help="what to allocate: nothing or 1 "
                                      "requests 1 SpiNN-5 board, NUM requests "
                                      "at least NUM SpiNN-5 boards, WIDTH "
                                      "HEIGHT means WIDTHxHEIGHT triads of "
                                      "SpiNN-5 boards and X Y Z requests a "
                                      "board the specified logical board "
                                      "coordinate.")
    allocation_args.add_argument("--resume", "-r", type=int,
                                 help="if given, resume keeping the "
                                      "specified job alive rather than "
                                      "creating a new job (all allocation "
                                      "requirements will be ignored)")
    allocation_args.add_argument("--machine", "-m", nargs="?",
                                 default=cfg["machine"],
                                 help="only allocate boards which are part "
                                      "of a specific machine, or any machine "
                                      "if no machine is given "
                                      "(default: %(default)s)")
    allocation_args.add_argument("--tags", "-t", nargs="*", metavar="TAG",
                                 default=cfg["tags"] or ["default"],
                                 help="only allocate boards which have (at "
                                      "least) the specified flags "
                                      "(default: {})".format(
                                          " ".join(cfg["tags"] or [])))
    allocation_args.add_argument("--min-ratio", type=float, metavar="RATIO",
                                 default=cfg["min_ratio"],
                                 help="when allocating by number of boards, "
                                      "require that the allocation be at "
                                      "least as square as this ratio "
                                      "(default: %(default)s)")
    allocation_args.add_argument("--max-dead-boards", type=int, metavar="NUM",
                                 default=(-1 if cfg["max_dead_boards"] is None
                                          else cfg["max_dead_boards"]),
                                 help="boards allowed to be "
                                      "dead in the allocation, or -1 to allow "
                                      "any number of dead boards "
                                      "(default: %(default)s)")
    allocation_args.add_argument("--max-dead-links", type=int, metavar="NUM",
                                 default=(-1 if cfg["max_dead_links"] is None
                                          else cfg["max_dead_links"]),
                                 help="inter-board links allowed to be "
                                      "dead in the allocation, or -1 to allow "
                                      "any number of dead links "
                                      "(default: %(default)s)")
    allocation_args.add_argument("--require-torus", "-w", action="store_true",
                                 default=cfg["require_torus"],
                                 help="require that the allocation contain "
                                      "torus (a.k.a. wrap-around) "
                                      "links {}".format(
                                          "(default)" if cfg["require_torus"]
                                          else ""
                                      ))
    allocation_args.add_argument("--no-require-torus", "-W",
                                 action="store_false", dest="require_torus",
                                 help="do not require that the allocation "
                                      "contain torus (a.k.a. wrap-around) "
                                      "links {}".format(
                                          "" if cfg["require_torus"]
                                          else "(default)"
                                      ))

    command_args = parser.add_argument_group("command wrapping arguments")
    command_args.add_argument("--command", "-c", nargs=argparse.REMAINDER,
                              help="execute the specified command once boards "
                                   "have been allocated and deallocate the "
                                   "boards when the application exits ({} and "
                                   "{hostname} are substituted for the chip "
                                   "chip at (0, 0)'s hostname, {w} and "
                                   "{h} give the dimensions of the SpiNNaker "
                                   "machine in chips, {ethernet_ips} is a "
                                   "temporary file containing a CSV with "
                                   "three columns: x, y and hostname giving "
                                   "the hostname of each Ethernet connected "
                                   "SpiNNaker chip)")

    server_args = parser.add_argument_group("spalloc server arguments")

    server_args.add_argument("--owner", default=cfg["owner"],
                             help="by convention, the email address of the "
                                  "owner of the job (default: %(default)s)")
    server_args.add_argument("--hostname", "-H", default=cfg["hostname"],
                             help="hostname or IP of the spalloc server "
                                  "(default: %(default)s)")
    server_args.add_argument("--port", "-P", default=cfg["port"],
                             type=int,
                             help="port number of the spalloc server "
                                  "(default: %(default)s)")
    server_args.add_argument("--keepalive", type=int, metavar="SECONDS",
                             default=(-1 if cfg["keepalive"] is None
                                      else cfg["keepalive"]),
                             help="the interval at which to require "
                                  "keepalive messages to be sent to "
                                  "prevent the server cancelling the "
                                  "job, or -1 to not require keepalive "
                                  "messages (default: %(default)s)")
    server_args.add_argument("--reconnect-delay",
                             default=cfg["reconnect_delay"],
                             type=float, metavar="SECONDS",
                             help="seconds to wait before "
                                  "reconnecting to the server if the "
                                  "connection is lost (default: %(default)s)")
    server_args.add_argument("--timeout", default=cfg["timeout"],
                             type=float, metavar="SECONDS",
                             help="seconds to wait for a response "
                                  "from the server (default: %(default)s)")

    args = parser.parse_args(argv)

    # Fail if no owner is defined (unless resuming)
    if not args.owner and args.resume is None:
        parser.error(
            "--owner must be specified (typically your email address)")

    # Fail if server not specified
    if args.hostname is None:
        parser.error("--hostname of spalloc server must be specified")

    # Set universal job arguments
    job_kwargs = {
        "hostname": args.hostname,
        "port": args.port,
        "reconnect_delay":
            args.reconnect_delay if args.reconnect_delay >= 0.0 else None,
        "timeout": args.timeout if args.timeout >= 0.0 else None,
    }

    if args.resume:
        job_args = []
        job_kwargs.update({
            "resume_job_id": args.resume,
        })
    else:
        # Make sure 'what' takes the right form
        if len(args.what) not in (0, 1, 2, 3):
            parser.error("expected either no arguments, one argument, NUM, "
                         "two arguments, WIDTH HEIGHT, or three arguments "
                         "X Y Z")

        # Unpack arguments for the job and server
        job_args = args.what
        job_kwargs.update({
            "owner": args.owner,
            "keepalive": args.keepalive if args.keepalive >= 0.0 else None,
            "machine": args.machine,
            "tags": args.tags if args.machine is None else None,
            "min_ratio": args.min_ratio,
            "max_dead_boards":
                args.max_dead_boards if args.max_dead_boards >= 0.0 else None,
            "max_dead_links":
                args.max_dead_links if args.max_dead_links >= 0.0 else None,
            "require_torus": args.require_torus,
        })

    # Set debug level
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    # Create temporary file in which to write CSV of all board IPs
    _, ip_file_filename = tempfile.mkstemp(".csv", "spinnaker_ips_")

    def info(msg):
        if not args.quiet:
            t.stream.write("{}\n".format(msg))

    # Reason for destroying the job
    reason = None

    try:
        # Create the job
        try:
            job = Job(*job_args, **job_kwargs)
        except (OSError, IOError) as e:
            info(t.red("Could not connect to server: {}".format(e)))
            return 6
        try:
            # Wait for it to become ready, keeping the user informed along the
            # way
            old_state = None
            cur_state = job.state
            while True:
                # Show debug info on state-change
                if old_state != cur_state:
                    if cur_state == JobState.queued:
                        info(t.update(t.yellow(
                            "Job {}: Waiting in queue...".format(job.id))))
                    elif cur_state == JobState.power:
                        info(t.update(t.yellow(
                            "Job {}: Waiting for power on...".format(job.id))))
                    elif cur_state == JobState.ready:
                        # Here we go!
                        break
                    elif cur_state == JobState.destroyed:
                        # Exit with error state
                        try:
                            reason = job.reason
                        except (IOError, OSError):
                            reason = None

                        if reason is not None:
                            info(t.update(t.red(
                                "Job {}: Destroyed: {}".format(
                                    job.id, reason))))
                        else:
                            info(t.red("Job {}: Destroyed.".format(job.id)))
                        return 1
                    elif cur_state == JobState.unknown:
                        info(t.update(t.red(
                            "Job {}: Job not recognised by server.".format(
                                job.id))))
                        return 2
                    else:
                        info(t.update(t.red(
                            "Job {}: Entered an unrecognised state {}.".format(
                                job.id, cur_state))))
                        return 3

                try:
                    old_state = cur_state
                    cur_state = job.wait_for_state_change(cur_state)
                except KeyboardInterrupt:
                    # Gracefully terminate from keyboard interrupt
                    info(t.update(t.red(
                        "Job {}: Keyboard interrupt.".format(
                            job.id))))
                    reason = "Keyboard interrupt."
                    return 4

            # Machine is now ready
            write_ips_to_csv(job.connections, ip_file_filename)

            # Boot the machine if required
            if MachineController is not None and args.boot:
                info(t.update(t.yellow(
                    "Job {}: Booting...".format(job.id))))
                mc = MachineController(job.hostname)
                mc.boot(job.width, job.height)

            info(t.update(t.green("Job {}: Ready!".format(job.id))))

            # Either run the user's application or just print the details.
            if args.command:
                return run_command(args.command, job.id, job.machine_name,
                                   job.connections, job.width, job.height,
                                   ip_file_filename)

            else:
                print_info(job.machine_name, job.connections,
                           job.width, job.height, ip_file_filename)
                return 0
        finally:
            # Destroy job and disconnect client
            if args.no_destroy:
                job.close()
            else:
                job.destroy(reason)
    finally:
        # Delete IP address list file
        os.remove(ip_file_filename)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
