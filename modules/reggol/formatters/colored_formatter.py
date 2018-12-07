import logging

from reggol.formatters.bold_formatter import BoldFormatter


class ColoredFormatter(BoldFormatter):
    COLOR_SEQ = "\033[1;%dm"

    BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)
    COLORS = {
        'WARNING': YELLOW,
        'INFO': GREEN,
        'DEBUG': BLUE,
        'CRITICAL': MAGENTA,
        'ERROR': RED
    }

    GROUP_COLOR = CYAN

    FORMAT = "[%(levelname)-5s] %(message)s".replace("$RESET", BoldFormatter.RESET_SEQ).replace("$BOLD", BoldFormatter.BOLD_SEQ)

    def __init__(self, log_format=FORMAT, colors=None):
        super().__init__(log_format)
        if colors is None:
            colors = self.COLORS
        self.formatter = logging.Formatter
        self.colors = colors

    def add_level(self, level: int, level_name: str, **kwargs):
        logging.addLevelName(level, level_name)
        self.colors[level_name] = kwargs['color']

    def format(self, record):
        levelname = record.levelname
        try:
            msg = record.msg.split(self.separator, 1)
            if len(msg) > 1:
                msg[0] = f'{record.name}:{msg[0]}'
            else:
                msg.insert(0, record.name)
            record.msg = f'[\x1b[1;3{self.GROUP_COLOR}m%s\x1b[0m] %s' % (msg[0], ' '.join(msg[1:]))
        except:
            pass

        if levelname in self.colors:
            levelname_color = self.COLOR_SEQ % (30 + self.colors[levelname]) + levelname + self.RESET_SEQ
            record.levelname = levelname_color

        return self.formatter.format(self, record)
