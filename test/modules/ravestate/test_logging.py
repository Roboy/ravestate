from ravestate.logger import get_logger

LOGGER_NAME = 'test_logger'


def test_get_logger():
    logger = get_logger(LOGGER_NAME)
    from reggol import CustomConsoleAndFileLogger
    assert isinstance(logger, CustomConsoleAndFileLogger)
