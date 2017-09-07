"""Utilities for generating ASCII/ANSI terminal graphics."""

import os
import sys

from functools import partial
from itertools import chain
from collections import defaultdict

from six import iteritems, string_types

from enum import IntEnum


class ANSIDisplayAttributes(IntEnum):
    """Code numbers of ANSI display attributes for use with ESC[...m sequences.
    """

    reset = 0
    bright = 1
    dim = 2
    underscore = 4
    blink = 5
    reverse = 7
    hidden = 8

    # foreground colours
    black = 30
    red = 31
    green = 32
    yellow = 33
    blue = 34
    magenta = 35
    cyan = 36
    white = 37

    # background colours
    bg_black = 40
    bg_red = 41
    bg_green = 42
    bg_yellow = 43
    bg_blue = 44
    bg_magenta = 45
    bg_cyan = 46
    bg_white = 47


class Terminal(object):
    """ANSI terminal control shenanigans.

    Utilities for printing colourful output and re-printing the screen on ANSI
    terminals. When output is not directed to a TTY, or when running under
    Windows, no ANSI control characters are produced.

    Examples::

        t = Terminal()

        # Printing in colours
        print(t.red("I'm in red!"))

        # Updating a status line
        for num in range(100):
            print(t.update("Now at {}%".format(num)))
            time.sleep(0.05)

        # Combining style attributes
        print(t.bg_red_white_blink("Woah!"))

    This module was inspired by the 'blessings' module which I initially liked
    but proved to be just a little too buggy.

    Attributes
    ----------
    stream
        The IO stream which is being used.
    enabled : bool
        Is colour enabled?
    """

    def __init__(self, stream=None, force=None):
        """
        Parameters
        ----------
        stream
            The IO stream being written to (by default sys.stdout).
        force : None or bool
            If a bool, forces styling to be enabled or disabled as specified.
            If None, checks whether the stream is a TTY (and that we're not o
            non-posix OS) before enabling colouring automatically.
        """
        self.stream = stream if stream is not None else sys.stdout

        if force is None:
            self.enabled = os.name == "posix" and self.stream.isatty()
        else:
            self.enabled = force

        self._location_saved = False

    def __call__(self, string):
        """If enabled, passes through the given value, otherwise passes through
        an empty string.
        """
        if self.enabled:
            return string
        else:
            return ""

    def clear_screen(self):
        """Clear the screen and reset cursor to top-left corner."""
        return self("\033[2J\033[;H")

    def update(self, string="", start_again=False):
        """Print before a line and it will replace the previous line prefixed
        with :py:meth:`.update`.

        Parameters
        ----------
        string : str
            The string to print (optional).
        start_again : bool
            If False, overwrites the last thing printed. If True, starts a new
            line.
        """
        if start_again:
            self._location_saved = False

        if not self._location_saved:
            # No previous line to update, just save the cursor.
            self._location_saved = True
            return "".join((self("\0337"), str(string)))
        else:
            # Restore to previous location and clear line.
            return "".join((self("\0338\033[K"), str(string)))

    def set_attrs(self, attrs=tuple()):
        """Construct an ANSI control sequence which sets the given attribute
        numbers.
        """
        if attrs:
            return self("\033[{}m".format(";".join(str(attr)
                                                   for attr in attrs)))
        else:
            return ""

    def wrap(self, string=None, pre="", post=""):
        """Wrap a string in the suppled pre and post strings or just print the
        pre string if no string given.
        """
        if string is not None:
            return "".join((pre, str(string), post))
        else:
            return pre

    def __getattr__(self, name):
        """Implements all the 'magic' style methods."""
        attrs = []
        while name:
            for attr in ANSIDisplayAttributes:
                if name.startswith(attr.name):
                    attrs.append(int(attr))
                    name = name[len(attr.name):].lstrip("_")
                    break
            else:
                # No attr name matched! Fail!
                raise AttributeError(name)
        return partial(self.wrap,
                       pre=self.set_attrs(attrs),
                       post=self("\033[0m"))


def render_table(table, column_sep="  "):
    """Render an ASCII table with optional ANSI escape codes.

    An example table::

        Something   Another thing  Finally
        some value  woah              1234
        ace         duuued              -1
        magic       rather good       9001

    Parameters
    ----------
    table : [row, ...]
        A table to render. Each row contains an iterable of column values which
        may be either values or a tuples (f, value) where value is the string
        to print, or an integer to print right-aligned. If a column is a tuple,
        f is a formatting function which is applied to the string before the
        table is finally displayed. Columns are padded to have matching widths
        *before* any formatting functions are applied.
    column_sep : str
        String inserted between each column.

    Returns
    -------
    str
        The formatted table.
    """
    # Determine maximum column widths
    column_widths = defaultdict(lambda: 0)
    for row in table:
        for i, column in enumerate(row):
            if isinstance(column, string_types):
                string = column
            elif isinstance(column, int):
                string = str(column)
            else:
                _, string = column
            column_widths[i] = max(len(str(string)), column_widths[i])

    # Render the table cells with padding [[str, ...], ...]
    out = []
    for row in table:
        rendered_row = []
        out.append(rendered_row)
        for i, column in enumerate(row):
            # Get string length and formatted string
            if isinstance(column, string_types):
                string = column
                length = len(string)
                right_align = False
            elif isinstance(column, int):
                string = str(column)
                length = len(string)
                right_align = True
            elif isinstance(column[1], string_types):
                f, string = column
                length = len(string)
                right_align = False
                string = f(string)
            elif isinstance(column[1], int):
                f, string = column
                length = len(str(string))
                right_align = True
                string = f(string)

            padding = " " * (column_widths[i] - length)
            if right_align:
                rendered_row.append(padding + string)
            else:
                rendered_row.append(string + padding)

    # Render the final table
    return "\n".join(column_sep.join(row).rstrip() for row in out)


def render_definitions(definitions, seperator=": "):
    """Render a definition list.

    Such a list looks like this::

              Key: Value
        Something: Else
          Another: Thing,
                   but this time with
                   line
                   breaks!

    Parameters
    ----------
    definitions : :py:class:`collections.OrderedDict`
        The key/value set to display.
    seperator : str
        The seperator inserted between keys and values.
    """
    # Special case since max would fail
    if not definitions:
        return ""

    col_width = max(map(len, definitions))
    return "\n".join("{:>{}s}{}{}".format(
        key, col_width, seperator, str(value).replace(
            "\n", "\n{}".format(" "*(col_width + len(seperator)))))
        for key, value in iteritems(definitions))


def _board_to_cartesian(x, y, z):
    """Translate from logical board coordinates (x, y, z) into cartesian
    coordinates for printing hexagons.

    Example coordinates::

         ___     ___
        /-15\___/1 5\___
        \___/0 4\___/3 4\
        /-13\___/1 3\___/
        \___/0 2\___/2 2\___
            \___/1 1\___/3 1\
            /0 0\___/2 0\___/
            \___/   \___/

    Parameters
    ----------
    x, y, z : int
        The logical board coordinates.

    Returns
    -------
    x, y : int
        Equivalent Cartesian coordinates.
    """
    cx = (2*x) - y + (1 if z == 1 else 0)
    cy = (3*y) + z

    return (cx, cy)

_LINK_TO_EDGE = {
    0: (+1, -1, 2),  # E
    1: (+1, +0, 1),  # NE
    2: (+0, +1, 0),  # N
    3: (+0, +0, 2),  # W
    4: (+0, -1, 1),  # SW
    5: (+0, -1, 0),  # S
}
r"""Mapping from link direction to board edge.

We define and number a board's link directions as::

         N 2
         ___
     3 W/   \ NE 1
    4 SW\___/ E 0
         S 5

Boards have the following *three* edge numbers::

      ___
    2/   \
    1\___/
       0

This means that, for example, the North 'edge' is actually represented as a
South edge on the board above. As a result this lookup table maps each link
direction to both a delta (in cartesian coordinates) and edge number.

{link: (dx, dy, edge), ...}
"""

_LINK_TO_DELTA = {
    0: (+1, -1),  # E
    1: (+1, +1),  # NE
    2: (+0, +2),  # N
    3: (-1, +1),  # W
    4: (-1, -1),  # SW
    5: (-0, -2),  # S
}
"""The cartesian offsets of the immediate neighbouring boards."""


DEFAULT_BOARD_EDGES = ("___", "\\", "/")
"""The default board edge styles."""


def render_boards(board_groups, dead_links=set(),
                  dead_edge=("XXX", "X", "X"),
                  blank_label="   ", blank_edge=("   ", " ", " ")):
    r"""Render an ASCII art diagram of a set of boards with sets of boards.

    For example::

         ___     ___     ___
        / . \___/ . \___/ . \___
        \___/ . \___/ . \___/ . \
        / . \___/ . \___/ . \___/
        \___/   \___/   \___/

    Parameters
    ----------
    board_groups : [([(x, y, z), ...], label, edge_inner, edge_outer), ...]
        Lists the groups of boards to display. Label is a 3-character string
        labelling the boards in the group, edge_inner and edge_outer are the
        characters to use to draw board edges as a tuple ("___", "\\", "/")
        which are to be used for the inner and outer board edges repsectively.
        Board groups are drawn sequentially with later board groups obscuring
        earlier ones when their edges or boards overlap.
    dead_links : set([(x, y, z, link), ...])
        Enumeration of all dead links. These links are re-drawn in the style
        defined by the dead_edge argument after all board groups have been
        drawn.
    dead_edge : ("___", "\\", "/")
        The strings to use to draw dead links.
    blank_label : "   "
        The 3-character string to use to label non-existant boards. (Blank by
        default)
    blank_edge : ("___", "\\", "/")
        The characters to use to render non-existant board edges. (Blank by
        default)
    """
    # {(x, y): string_types, ...}
    board_labels = {}
    # {(x, y, edge): str, ...}
    board_edges = {}

    # The set of all boards defined (used to filter displaying of dead links to
    # non-existant boards
    all_boards = set()

    for boards, label, edge_inner, edge_outer in board_groups:
        # Convert to cartesian coords
        boards = set(_board_to_cartesian(x, y, z) for x, y, z in boards)
        all_boards.update(boards)

        # Set board labels and basic edge style
        for x, y in boards:
            board_labels[(x, y)] = label

            for link in range(6):
                dx, dy = _LINK_TO_DELTA[link]
                x2 = x + dx
                y2 = y + dy

                edx, edy, edge = _LINK_TO_EDGE[link]
                if (x2, y2) in boards:
                    style = edge_inner[edge]
                else:
                    style = edge_outer[edge]
                ex = x + edx
                ey = y + edy
                board_edges[(ex, ey, edge)] = style

    # Mark dead links
    for x, y, z, link in dead_links:
        x, y = _board_to_cartesian(x, y, z)
        edx, edy, edge = _LINK_TO_EDGE[link]
        ex = x + edx
        ey = y + edy
        board_edges[(ex, ey, edge)] = dead_edge[edge]

    # Get the bounds of the size of diagram to render
    all_xy = tuple(chain(all_boards, ((x, y) for x, y, edge in board_edges)))
    if len(all_xy) == 0:
        return ""  # Special case since min/max will fail otherwise
    x_min, y_min = map(min, zip(*all_xy))
    x_max, y_max = map(max, zip(*all_xy))

    # Render row-by-row
    #   ___     ___            6 Even
    #  /-15\___/1 5\___        5 Odd
    #  \___/0 4\___/3 4\       4 Even
    #  /-13\___/1 3\___/       3 Odd
    #  \___/0 2\___/2 2\___    2 Even
    #  .   \___/1 1\___/3 1\   1 Odd
    #  .   /0 0\___/2 0\___/   0 Even
    #  .   \___/   \___/      -1 Odd
    # -1   0   1   2   3   4
    #  Odd Evn Odd Evn Odd Evn
    out = []
    for y in range(y_max, y_min - 1, -1):
        even_row = (y % 2) == 0
        row = ""
        for x in range(x_min, x_max + 1):
            even_col = (x % 2) == 0
            if even_row == even_col:
                row += board_edges.get((x, y, 2), blank_edge[2])
                row += board_labels.get((x, y), blank_label)
            else:
                row += board_edges.get((x, y, 1), blank_edge[1])
                row += board_edges.get((x, y, 0), blank_edge[0])
        out.append(row)

    return "\n".join(filter(None, map(str.rstrip, out)))


def render_cells(cells, width=80, col_spacing=2):
    """Given a list of short (~10 char) strings, display these aligned in
    columns.

    Example output::

        Something  like       this       can        be
        used       to         neatly     arrange    long
        sequences  of         values     in         a
        compact    format.

    Parameters
    ----------
    cells : [(strlen, str), ...]
        Gives the cells to print as tuples giving the strings length in visible
        characters and the string to display.
    width : int
        The width of the terminal.
    col_spacing : int
        Size of the gap to leave between columns.
    """
    # Special case (since max below will fail)
    if len(cells) == 0:
        return ""

    # Columns should be at least as large as the largest cell with padding
    # between columns
    col_width = max(strlen for strlen, s in cells) + col_spacing

    lines = [""]
    cur_length = 0
    for strlen, s in cells:
        # Once line is full, move to the next
        if cur_length + strlen > width:
            lines.append("")
            cur_length = 0

        # Add the current cell (with spacing)
        lines[-1] += s + (" "*(col_width - strlen))
        cur_length += col_width

    return "\n".join(map(str.rstrip, lines))
