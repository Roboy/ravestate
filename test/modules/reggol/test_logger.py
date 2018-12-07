import os
import time

import pytest
from testfixtures import log_capture

log_file_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'log')


@pytest.mark.serial
def test_file():
    pass