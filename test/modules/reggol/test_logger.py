import os

import pytest
from testfixtures import log_capture


LOGGER_NAME = 'test'
log_file_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'log')
log_file_name = 'test_log.log'


@pytest.fixture
def test_logger():
    from reggol import CustomConsoleAndFileLogger
    return CustomConsoleAndFileLogger(LOGGER_NAME)


@pytest.mark.serial
def test_file(test_logger):
    test_logger.set_file_formatter(log_file_dir, log_file_name)
    test_logger.info("Hello")
    file = open(f"{log_file_dir}/{log_file_name}", "r")
    log_content = file.read()
    assert log_content.find("Hello") > 0


@log_capture()
@pytest.mark.serial
def test_basic(capture, test_logger):
    test_logger.info('a message')
    test_logger.error('a message')
    test_logger.debug('a message')
    test_logger.warning('a message')
    test_logger.critical('a message')

    # capture.check(
    #     (LOGGER_NAME, '\x1b[1;32mINFO\x1b[0m', 'a message'),
    #     (LOGGER_NAME, '\x1b[1;31mERROR\x1b[0m', 'a message'),
    #     (LOGGER_NAME, '\x1b[1;34mDEBUG\x1b[0m', 'a message'),
    #     (LOGGER_NAME, '\x1b[1;33mWARNING\x1b[0m', 'a message'),
    #     (LOGGER_NAME, '\x1b[1;35mCRITICAL\x1b[0m', 'a message'),
    # )


@log_capture()
@pytest.mark.serial
def test_module(capture, test_logger):
    test_logger.info('module -> a message')

    pass
