import logging


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

    FORMAT = "[%(levelname_color)-5s] [%(name_color)s] %(msg)s"

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
        return super(ColoredFormatter, self).format(record)
