"""Command-line tool to find out where a particular chip or board resides.

The ``spalloc-where-is`` command allows you to query boards by coordinate, by
physical location, by chip or by job. In response to a query, a standard set of
information is displayed as shown in the example below::

    $ spalloc-where-is --job-chip 24 14, 3
                     Machine: my-machine
           Physical Location: Cabinet 2, Frame 4, Board 7
            Board Coordinate: (3, 4, 0)
    Machine Chip Coordinates: (38, 51)
    Coordinates within board: (2, 3)
             Job using board: 24
      Coordinates within job: (14, 3)

In this example we ask, 'where is chip (14, 3) in job 24'? We discover that:

* The chip is the machine named 'my-machine' on the board in cabinet 2, frame
  4, board 7.
* This board's logical board coordinates are (3, 4, 0). These logical
  coordinates may be used to specifically request this board from Spalloc in
  the future.
* If 'my-machine' were booted as a single large machine, the chip we queried
  would be chip (38, 51). This may be useful for cross-referencing with
  diagrams produced by SpiNNer_.
* The chip in question is chip (2, 3) its board. This may be useful when
  reporting faulty chips/replacing boards..
* The job currently running on the board has ID 24. Obviously in this example
  we already knew this but this may be useful when querying by board.
* Finally, we're told that the queried chip has the coordinates (14, 3) in the
  machine allocated to job 24. Again, this information may be more useful when
  querying by board.

.. _SpiNNer: https://github.com/SpiNNakerManchester/SpiNNer

To query by logical board coordinate::

    spalloc-where-is --board MACHINE X Y Z

To query by physical board location::

    spalloc-where-is --physical MACHINE CABINET FRAME BOARD

To query by chip coordinate (as if the machine were booted as one large
machine)::

    spalloc-where-is --chip MACHINE X Y

To query by chip coordinate of chips allocated to a job::

    spalloc-where-is --job-chip JOB_ID X Y
"""
import sys
import argparse

from collections import OrderedDict

from spalloc import config
from spalloc import __version__, ProtocolClient, ProtocolTimeoutError
from spalloc.term import render_definitions


# The acceptable range of server version numbers
VERSION_RANGE_START = (0, 3, 0)
VERSION_RANGE_STOP = (2, 0, 0)


def main(argv=None):
    cfg = config.read_config()

    parser = argparse.ArgumentParser(
        description="Find out the location (physical or logical) "
                    "of a chip or board.")

    parser.add_argument("--version", "-V", action="version",
                        version=__version__)

    control_args = parser.add_mutually_exclusive_group(required=True)
    control_args.add_argument("--board", "-b", "--logical", "-l", nargs=4,
                              metavar=("MACHINE", "X", "Y", "Z"),
                              help="specify the logical board coordinate")
    control_args.add_argument("--physical", "-p", nargs=4,
                              metavar=("MACHINE", "CABINET", "FRAME", "BOARD"),
                              help="specify a board's physical location")
    control_args.add_argument("--chip", "-c", nargs=3,
                              metavar=("MACHINE", "X", "Y"),
                              help="specify a board by chip coordinates (as "
                                   "if the whole machine is being used)")
    control_args.add_argument("--job-chip", "-j", nargs=3,
                              metavar=("JOB_ID", "X", "Y"),
                              help="specify the chip coordinates of a chip "
                                   "within a job's boards")

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

        # Work out what the user asked for
        try:
            show_board_chip = False
            if args.board:
                machine, x, y, z = args.board
                where_is_kwargs = {
                    "machine": machine,
                    "x": int(x),
                    "y": int(y),
                    "z": int(z),
                }
            elif args.physical:
                machine, c, f, b = args.physical
                where_is_kwargs = {
                    "machine": machine,
                    "cabinet": int(c),
                    "frame": int(f),
                    "board": int(b),
                }
            elif args.chip:
                machine, x, y = args.chip
                where_is_kwargs = {
                    "machine": machine,
                    "chip_x": int(x),
                    "chip_y": int(y),
                }
                show_board_chip = True
            elif args.job_chip:
                job_id, x, y = args.job_chip
                where_is_kwargs = {
                    "job_id": int(job_id),
                    "chip_x": int(x),
                    "chip_y": int(y),
                }
                show_board_chip = True
        except ValueError as e:
            parser.error("Error: {}".format(e))

        # Ask the server
        location = client.where_is(**where_is_kwargs)
        if location is None:
            sys.stderr.write("No boards at the specified location.\n")
            return 4
        else:
            out = OrderedDict()
            out["Machine"] = location["machine"]
            out["Physical location"] = "Cabinet {}, Frame {}, Board {}".format(
                *location["physical"])
            out["Board coordinate"] = tuple(location["logical"])
            out["Machine chip coordinates"] = tuple(location["chip"])
            if show_board_chip:
                out["Coordinates within board"] = tuple(location["board_chip"])
            out["Job using board"] = location["job_id"]
            if location["job_id"]:
                out["Coordinates within job"] = tuple(location["job_chip"])

            print(render_definitions(out))
            return 0

    except (IOError, OSError, ProtocolTimeoutError) as e:
        sys.stderr.write("Error communicating with server: {}\n".format(e))
        return 1
    finally:
        client.close()


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
