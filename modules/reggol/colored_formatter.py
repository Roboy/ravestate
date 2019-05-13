import logging
import sys


def get_stack_size():
    """Get stack size for caller's frame.

    %timeit len(inspect.stack())
    8.86 ms ± 42.5 µs per loop (mean ± std. dev. of 7 runs, 100 loops each)
    %timeit get_stack_size()
    4.17 µs ± 11.5 ns per loop (mean ± std. dev. of 7 runs, 100000 loops each)
    """
    size = 2  # current frame and caller's frame always exist
    while True:
        try:
            sys._getframe(size)
            size += 1
        except ValueError:
            return size - 1  # subtract current frame


class ColoredFormatter(logging.Formatter):
    COLOR_SEQ = "\033[1;%dm"
    BOLD_SEQ = "\033[1m"
    RESET_SEQ = "\033[0m"

    BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)
    COLORS = {
        'WARNING': YELLOW,
        'INFO': GREEN,
        'DEBUG': BLUE,
        'CRITICAL': MAGENTA,
        'ERROR': RED
    }

    GROUP_COLOR = CYAN

    FORMAT = "[%(threadName)s] %(stack_size)s %(name_color)s: [%(levelname_color)-5s] %(msg)s"

    def __init__(self, log_format=FORMAT, colors=None):
        super().__init__(log_format)
        if colors is None:
            colors = self.COLORS
        self.colors = colors

    def format(self, record):
        record.name_color = f'\x1b[1;3{self.GROUP_COLOR}m{record.name}\x1b[0m'
        levelname = record.levelname
        if levelname in self.colors:
            record.levelname_color = self.COLOR_SEQ % (30 + self.colors[levelname]) + levelname + self.RESET_SEQ
        else:
            record.levelname_color = record.levelname
        record.stack_size = "|" * max(get_stack_size()-10, 0)
        return super(ColoredFormatter, self).format(record)
