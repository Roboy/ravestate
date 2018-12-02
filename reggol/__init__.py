import argparse
import configparser
import os
import sys
from jinja2 import Template

from reggol.formatters.colored_formatter import ColoredFormatter
from reggol.formatters.mixed_formatter import MixedFormatter
from reggol.logger import CustomConsoleAndFileLogger


config = configparser.ConfigParser()
config.read(os.path.join(os.path.abspath(os.path.dirname(__file__)), 'config.ini'))

parser = argparse.ArgumentParser(description='Reggol - logging parameters:')
parser.add_argument('--log_level',  metavar='LEVEL', type=str,
                    help='Logging level', default=config.get('DEFAULT', 'level'))
parser.add_argument('--log_dir', metavar='DIR', type=str,
                    help='Log files directory name', default=config.get('DEFAULT', 'dir'))
args, unknown_args = parser.parse_known_args()

sys.argv[1:] = unknown_args

_level = args.log_level
_path = Template(args.log_dir).render(default_dir=os.path.dirname(__file__))


def get_logger(name: str):
    logger = CustomConsoleAndFileLogger(name, _level)
    logger.set_console_formatter(MixedFormatter(ColoredFormatter()))
    logger.set_file_formatter()
    return logger
