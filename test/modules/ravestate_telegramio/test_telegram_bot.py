from ravestate.context import Context
from ravestate.state import Delete
from ravestate.testfixtures import *
from pytest_mock import mocker
from testfixtures import LogCapture

from ravestate_telegramio.telegram_bot import *


def generate_fake_conf(conf_dict: Dict):
    def fake_conf(key, **kwargs):
        return conf_dict[key]
    return fake_conf


def test_telegram_run(mocker, context_wrapper_fixture: Context):
    with LogCapture(attributes=strip_prefix) as capture:
        assert isinstance(telegram_run(context_wrapper_fixture), Delete)
        capture.check_present('telegram-token is not set. Shutting down telegramio')

    with LogCapture(attributes=strip_prefix) as log_capture:
        # Master Process should return Delete and error for telegram output when all_in_one_context is True and no Token
        with mocker.patch.object(context_wrapper_fixture, "conf",
                                 side_effect=generate_fake_conf({CHILD_CONN_CONFIG_KEY: None,
                                                                 ALL_IN_ONE_CONTEXT_CONFIG_KEY: False,
                                                                 TOKEN_CONFIG_KEY: "1234",
                                                                 CHILD_FILES_CONFIG_KEY: ['/not/a/config/path']})):
            assert isinstance(telegram_run(context_wrapper_fixture), Delete)
            log_capture.check_present(f'{CHILD_FILES_CONFIG_KEY} is not set (correctly). Shutting down telegramio')
    # TODO: Write test for token present with patched Updater


def test_telegram_output(mocker, context_wrapper_fixture: ContextWrapper):
    with LogCapture(attributes=strip_prefix) as log_capture:
        # Master Process should return Delete and error for telegram output when all_in_one_context is True and no Token
        with mocker.patch.object(context_wrapper_fixture, "conf",
                                 side_effect=generate_fake_conf({ALL_IN_ONE_CONTEXT_CONFIG_KEY: True,
                                                                 TOKEN_CONFIG_KEY: ""})):
            assert isinstance(telegram_output(context_wrapper_fixture), Delete)
            log_capture.check_present('telegram-token is not set. Shutting down telegramio')

    # Child process should send message via pipe
    context_wrapper_fixture.spike_payloads = {"rawio:out:changed": "FAKE TEXT"}
    child_conn = mocker.patch('multiprocessing.connection.Connection')
    with mocker.patch.object(child_conn, "send"):
        with mocker.patch.object(context_wrapper_fixture, "conf",
                                 side_effect=generate_fake_conf({ALL_IN_ONE_CONTEXT_CONFIG_KEY: False,
                                                                 CHILD_CONN_CONFIG_KEY: child_conn})):
            telegram_output(context_wrapper_fixture)
            child_conn.send.assert_called_once_with("FAKE TEXT")

    # Master Process should return Delete and no error for telegram output when all_in_one_context is False
    with mocker.patch.object(context_wrapper_fixture, "conf",
                             side_effect=generate_fake_conf({ALL_IN_ONE_CONTEXT_CONFIG_KEY: False,
                                                             CHILD_CONN_CONFIG_KEY: None})):
        assert isinstance(telegram_output(context_wrapper_fixture), Delete)

    # TODO: Write test for token present


def test_timestamp(mocker):
    timemock = 42.27
    with mocker.patch("time.time", return_value=timemock):
        stamp = Timestamp()
        assert stamp.value == timemock
        assert stamp.age() == 0.
    new_timemock = timemock + 12.34
    with mocker.patch("time.time", return_value=new_timemock):
        assert stamp.age() == new_timemock - timemock
        stamp.update()
        assert stamp.value == new_timemock
        assert stamp.age() == 0.
