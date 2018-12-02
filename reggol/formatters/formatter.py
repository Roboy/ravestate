import logging

from reggol.formatters import SEPARATOR, FORMAT


class Formatter(logging.Formatter):
    separator = SEPARATOR

    def __init__(self, log_format=FORMAT):
        super().__init__(log_format)
        self.formatter = logging.Formatter

    @staticmethod
    def add_level(level: int, level_name: str, **kwargs):
        logging.addLevelName(level, level_name)

    def format(self, record):
        try:
            msg = record.msg.split(SEPARATOR, 1)
            if len(msg) > 1:
                msg[0] = f'{record.name}:{msg[0]}'
            else:
                msg.insert(0, record.name)
            record.msg = f'[%s] %s' % (msg[0], ' '.join(msg[1:]))
        except:
            pass

        return self.formatter.format(self, record)
