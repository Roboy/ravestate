from ravestate import registry
from ravestate_telegramio import telegram_bot

import ravestate_rawio

registry.register(
    name="telegramio",
    states=(telegram_bot.telegram_run, telegram_bot.telegram_output),
    config={
        telegram_bot.TOKEN_CONFIG_KEY: ""
    })
