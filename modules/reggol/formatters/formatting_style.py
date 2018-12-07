from enum import Enum

from reggol.formatters.bold_formatter import BoldFormatter
from reggol.formatters.colored_formatter import ColoredFormatter
from reggol.formatters.formatter import Formatter


class FormattingStyle(Enum):
    PLAIN = 0
    COLOR = 1
    BOLD = 2
    DEFAULT = PLAIN


formatter_map = {
    FormattingStyle.PLAIN : Formatter,
    FormattingStyle.COLOR: ColoredFormatter,
    FormattingStyle.BOLD: BoldFormatter
}


def get_formatter(style: FormattingStyle):
    return formatter_map[style.key]()
