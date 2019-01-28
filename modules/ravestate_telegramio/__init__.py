from ravestate.module import Module
from ravestate_telegramio import telegram_bot

import ravestate_rawio


CONFIG = {
    telegram_bot.TOKEN_CONFIG_KEY: ""
}

with Module(name="telegramio", config=CONFIG) as mod:

    mod.add(telegram_bot.telegram_run)
    mod.add(telegram_bot.telegram_output)
