import logging
import os
import time

from reggol.formatters.formatter import Formatter
from reggol.formatters.mixed_formatter import MixedFormatter

TIME_FORMAT = "%Y-%m-%d %H:%M:%S"
LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)-80s : FILE %(filename)s - %(lineno)d"

DEFAULT_FILE_NAME = f"log_{time.strftime('%Y%m%d_%H%M%S')}.log"
DEFAULT_DIRECTORY = os.path.join(os.path.dirname(__file__), 'log')
DEFAULT_LEVEL = logging.INFO
CUSTOM_FORMAT_LEVEL = ''
CUSTOM_FORMAT_LEVEL_NAME = ''


class CustomConsoleAndFileLogger(logging.Logger):

    def __init__(self, name: str):
        super().__init__(name)
        self._console_formatter = None
        self._file_formatter = None

    def set_file_formatter(
            self,
            file_path: str = DEFAULT_DIRECTORY,
            file_name: str = DEFAULT_FILE_NAME,
            formatter: logging.Formatter = Formatter()
    ):
        self._file_formatter = formatter
        path = os.path.join(file_path, file_name)
        file = logging.FileHandler(path)
        file.setFormatter(formatter)
        self.addHandler(file)

    def set_console_formatter(self, formatter: logging.Formatter = MixedFormatter()):
        self._console_formatter = formatter

        console = logging.StreamHandler()
        console.setFormatter(formatter)
        self.addHandler(console)

    def add_level(self, level: int, level_name: str, **kwargs):
        self._file_formatter.addLevelName(level, level_name, **kwargs)
        self._console_formatter.addLevelName(level, level_name, **kwargs)

    def plain(self, msg, *kwargs):
        self.log(CUSTOM_FORMAT_LEVEL, msg, *kwargs)
