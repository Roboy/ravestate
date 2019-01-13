from reggol.logger import CustomConsoleAndFileLogger


def get_logger(name: str = __file__):
    logger = CustomConsoleAndFileLogger(name)
    logger.set_file_formatter()
    logger.set_console_formatter()
    return logger
