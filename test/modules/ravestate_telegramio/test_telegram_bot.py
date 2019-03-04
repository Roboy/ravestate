from ravestate.context import Context
from ravestate.state import Delete
from ravestate.testfixtures import *
from pytest_mock import mocker
from testfixtures import LogCapture

from ravestate_telegramio.telegram_bot import *

FILE_NAME = 'ravestate_telegramio.telegram_bot'
PREFIX = f"[{FILE_NAME}] [\x1b[1;36m{FILE_NAME}\x1b[0m]"


def test_telegram_run(context_wrapper_fixture: Context):
    with LogCapture() as capture:
        telegram_run(context_wrapper_fixture)
        expected = 'telegram-token is not set. Shutting down telegramio'
        capture.check_present((f"{FILE_NAME}", '\x1b[1;31mERROR\x1b[0m', f"{PREFIX} {expected}"))
        # TODO: Write test for token present


def test_telegram_output(context_wrapper_fixture: ContextWrapper):
    # Master Process should return delete for telegram output
    assert isinstance(telegram_output(context_wrapper_fixture), Delete)
    # TODO: Write test for token present

