import argparse
import os
import sys

from reggol.formatters.colored_formatter import ColoredFormatter
from reggol.formatters.mixed_formatter import MixedFormatter
from reggol.logger import CustomConsoleAndFileLogger
import logging

_level = None
_default_logpath: str = None


def set_default_loglevel(loglevel: str) -> None:
    """
    Set the default loglevel. Should be one of the loglevels
     known by python's logging.
    :param loglevel: The name of the loglevel.
    """
    global _level
    _level = loglevel


def set_default_logpath(logpath: str) -> None:
    """
    Set the default logger for all subsequent get_logger calls.
    :param logpath: The directory under which to store logfiles
     by default. Will be created if it doesn't exist.
    """
    global _default_logpath
    if not os.path.exists(logpath):
        os.makedirs(logpath)
    assert os.path.isdir(logpath)
    _default_logpath = logpath


def get_logger(name: str, logpath: str=None):
    """
    Obtain a file/console logger with a certain name.
    :param name: The name for the logger. Will appear in all log messages.
    :param logpath: The path under which a logfile should be created.
     If no path is specified, a default path is used.
     The default path is the working directory if no other path was specified
     by _any_ other reggol client using set_default_logpath(...)!
    :return: A new logger with the given name.
    """
    global _default_logpath
    if not logpath:
        logpath = _default_logpath
    logging.setLoggerClass(CustomConsoleAndFileLogger)
    logger = logging.getLogger(name)
    assert isinstance(logger, CustomConsoleAndFileLogger)
    logger.setLevel(_level)
    logger.set_console_formatter(MixedFormatter(ColoredFormatter()))
    logger.set_file_formatter(file_path=logpath)
    return logger


# Instantiate and configure argparser
argparser = argparse.ArgumentParser(description='Reggol -- logging options:', add_help=False)
argparser.add_argument('--loglevel', '-ll', metavar='LEVEL', type=str,
                       help='Logging level.', default="INFO", choices=logging._nameToLevel.keys())
argparser.add_argument('--logdir', '-ld', metavar='DIR', type=str,
                       help='Log files directory path.', default="log")


def help_string() -> str:
    """
    Get a string which describes reggols command line arguments.
    :return: A descriptive help text you can add to your help epilogue.
    """
    return "\n".join(argparser.format_help().split("\n")[2:])


def strip_prefix(lr: logging.LogRecord) -> str:
    """
    Split the prefix from a logging record, such that the
     original message may be recovered for tests.
    :return: The original message from the log record,
     without `[module] [level] ` prefix.
    """
    return "".join(lr.msg.split("] ")[2:])

args, unknown_args = argparser.parse_known_args()
sys.argv[1:] = unknown_args
set_default_loglevel(args.loglevel)
set_default_logpath(args.logdir)
