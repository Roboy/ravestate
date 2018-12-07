import logging

from reggol.formatters.colored_formatter import ColoredFormatter
from reggol.formatters.formatter import Formatter
from reggol.formatters.formatting_style import get_formatter


class MixedFormatter(Formatter):

    def __init__(self, default_formatter: logging.Formatter = ColoredFormatter(), formatters: dict = None):
        super().__init__()
        if formatters is None:
            formatters = {}
        self.formatter_map = formatters
        self.formatter = default_formatter

    def add_level(self, level: int, level_name: str, **kwargs):
        logging.addLevelName(level, level_name)
        self.formatter_map[level_name] = get_formatter(kwargs['formatting_style'])

    def format(self, record):
        try:
            if record.level in self.formatter_map:
                return self.formatter_map[record.level].format(record)
            else:
                self.formatter.format(record)
        except:
            pass

        return self.formatter.format(record)
