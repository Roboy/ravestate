import os
import time

import pytest
from testfixtures import log_capture

LOG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'log')


@pytest.mark.serial
def test_file():
    from ravestate.logger import Logger
    import fnmatch
    log_files = [f for f in os.listdir(LOG_PATH) if os.path.isfile(os.path.join(LOG_PATH, f))
                 and fnmatch.fnmatch(f, f"{time.strftime('log_%Y%m%d_%H%M')}*.log")]
    assert len(log_files) > 0

    Logger.info("Hello")
    file = open(f"{LOG_PATH}/{log_files[0]}", "r")
    log_content = file.read()
    assert log_content.find("Hello") > 0


@log_capture()
@pytest.mark.serial
def test_basic(capture):
    from ravestate.logger import Logger
    Logger.info('a message')
    Logger.error('a message')
    Logger.debug('a message')
    Logger.warning('a message')
    Logger.critical('a message')

    capture.check(
        ('ravestate', '\x1b[1;32mINFO\x1b[0m', 'a message'),
        ('ravestate', '\x1b[1;31mERROR\x1b[0m', 'a message'),
        ('ravestate', '\x1b[1;34mDEBUG\x1b[0m', 'a message'),
        ('ravestate', '\x1b[1;33mWARNING\x1b[0m', 'a message'),
        ('ravestate', '\x1b[1;35mCRITICAL\x1b[0m', 'a message'),
    )
