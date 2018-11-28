import logging

from ravestate.logger import ColorConsoleAndFileLogger

logging.setLoggerClass(ColorConsoleAndFileLogger)
Logger = logging.getLogger('ravestate')
