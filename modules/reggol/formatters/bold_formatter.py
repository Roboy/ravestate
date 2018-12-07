import logging

from reggol.formatters.formatter import Formatter


class BoldFormatter(Formatter):
    RESET_SEQ = "\033[0m"
    BOLD_SEQ = "\033[1m"

    FORMAT = "[%(levelname)-5s] %(message)s".replace("$RESET", "").replace("$BOLD", "")

    def __init__(self, log_format=FORMAT):
        super().__init__(log_format)
        self.formatter = logging.Formatter

    def format(self, record):
        try:
            msg = record.msg.split(self.separator, 1)
            if len(msg) > 1:
                msg[0] = f'{record.name}:{msg[0]}'
            else:
                msg.insert(0, record.name)
            record.msg = f'[\x1b%s\x1b[0m] %s' % (msg[0], ' '.join(msg[1:]))
        except:
            pass

        return self.formatter.format(self, record)

    @staticmethod
    def add_level(level: int, level_name: str, **kwargs):
        logging.addLevelName(level, level_name)
