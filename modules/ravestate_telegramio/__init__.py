from ravestate.module import Module
from ravestate_telegramio import telegram_bot


CONFIG = {
    telegram_bot.TOKEN_CONFIG_KEY: "",
    telegram_bot.CHILD_CONN_CONFIG_KEY: None,
    telegram_bot.CHILD_FILES_CONFIG_KEY: [],
    telegram_bot.ALL_IN_ONE_CONTEXT_CONFIG_KEY: False
}

with Module(name=telegram_bot.MODULE_NAME, config=CONFIG) as mod:

    mod.add(telegram_bot.telegram_run)
    mod.add(telegram_bot.telegram_output)
