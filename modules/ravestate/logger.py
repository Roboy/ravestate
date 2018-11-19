import logging
import os
import time

_SEPARATOR = '->'

BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)
_RESET_SEQ = "\033[0m"
_COLOR_SEQ = "\033[1;%dm"
_BOLD_SEQ = "\033[1m"

_LOG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'log')
_COLORS = {
    'WARNING': YELLOW,
    'INFO': GREEN,
    'DEBUG': BLUE,
    'CRITICAL': MAGENTA,
    'ERROR': RED
}


def formatter_message(message, use_color=True):
    if use_color:
        message = message.replace("$RESET", _RESET_SEQ).replace("$BOLD", _BOLD_SEQ)
    else:
        message = message.replace("$RESET", "").replace("$BOLD", "")
    return message


class ColoredFormatter(logging.Formatter):
    def __init__(self, msg, use_color=True):
        logging.Formatter.__init__(self, msg)
        self.use_color = use_color

    def format(self, record):
        levelname = record.levelname
        try:
            msg = record.msg.split(_SEPARATOR, 1)
            if len(msg) == 2:
                record.msg = f'[\x1b[1;3{CYAN}m%s\x1b[0m]%s' % (msg[0], msg[1])
        except:
            pass
        if self.use_color and levelname in _COLORS:
            levelname_color = _COLOR_SEQ % (30 + _COLORS[levelname]) + levelname + _RESET_SEQ
            record.levelname = levelname_color
        return logging.Formatter.format(self, record)


# Custom logger class with multiple destinations
class ColorConsoleAndFileLogger(logging.Logger):
    FORMAT = "[%(levelname)-20s]  %(message)s"
    COLOR_FORMAT = formatter_message(FORMAT, True)

    def __init__(self, name):
        logging.Logger.__init__(self, name, logging.DEBUG)

        file_name = f"log_{time.strftime('%Y%m%d_%H%M%S')}.log"
        path = os.path.join(_LOG_PATH, file_name)
        file = logging.FileHandler(path)
        file_formatter = logging.Formatter("%(asctime)s - %(levelname)s - "
                                           "%(message)-80s : FILE %(filename)s - %(lineno)d", "%Y-%m-%d %H:%M:%S")

        file.setFormatter(file_formatter)
        self.addHandler(file)

        color_formatter = ColoredFormatter(self.COLOR_FORMAT)
        console = logging.StreamHandler()
        console.setFormatter(color_formatter)
        self.addHandler(console)
        return


logging.setLoggerClass(ColorConsoleAndFileLogger)
Logger = logging.getLogger('ravestate')
Logger.info(os.path.dirname(os.path.abspath(__file__)))
