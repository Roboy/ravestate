import logging
import os

from reggol.formatters.colored_formatter import ColoredFormatter
from reggol.formatters.mixed_formatter import MixedFormatter
from reggol.logger import CustomConsoleAndFileLogger


_level = logging.INFO


def init_module(level: str, log_path: str):
    _level = level
    _path = log_path


def get_logger(name: str):
    logger = CustomConsoleAndFileLogger(name)
    logger.set_console_formatter(MixedFormatter(ColoredFormatter()))
    logger.set_file_formatter()
    return logger
