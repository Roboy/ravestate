import os
from typing import Set

from ravestate.constraint import s
from ravestate.receptor import receptor
from ravestate.state import state, Delete
from ravestate.wrappers import ContextWrapper
from telegram import Bot, Update, TelegramError
from telegram.ext import Updater, MessageHandler, Filters, Dispatcher
import logging
from yaml import load


TOKEN_CONFIG_KEY: str = "telegram-token"
active_chats: Set[int] = set()


@state(triggers=s(":startup"))
def telegram_run(ctx: ContextWrapper):
    """
    Starts up the telegram bot and adds a handler to write incoming messages to rawio:in
    """

    @receptor(ctx_wrap=ctx, write="rawio:in")
    def telegram_callback(ctx: ContextWrapper, message_text: str):
        """
        Writes the message_text to rawio:in
        """
        ctx["rawio:in"] = message_text

    def handle_input(bot: Bot, update: Update):
        """
        Handler for incoming messages
        Adds the chat of the incoming message to the set of active_chats
        Calls the telegram_callback receptor to process the incoming message
        """
        active_chats.add(update.message.chat_id)
        telegram_callback(update.message.text)

    def error(bot: Bot, update: Update, error: TelegramError):
        """
        Log Errors caused by Updates.
        """
        logging.warning('Update "%s" caused error "%s"', update, error)

    """Start the bot."""
    # Create the EventHandler and pass it your bots token.
    token = ctx.conf(key=TOKEN_CONFIG_KEY)
    if not token:
        logging.error('telegram-token is not set. Shutting down telegramio')
        return Delete()

    updater: Updater = Updater(token)
    # Get the dispatcher to register handlers
    dp: Dispatcher = updater.dispatcher
    # handle noncommand-messages with handle_input
    dp.add_handler(MessageHandler(Filters.text, handle_input))
    # log all errors
    dp.add_error_handler(error)
    # Start the Bot
    updater.start_polling()


@state(read="rawio:out")
def telegram_output(ctx: ContextWrapper):
    """
    Sends the content of rawio:out to every currently active chat
    """
    # TODO don't instantiate the updater every time
    token = ctx.conf(key=TOKEN_CONFIG_KEY)
    if not token:
        logging.error('telegram-token is not set. Shutting down telegramio')
        return Delete()

    updater: Updater = Updater(token)
    for chat_id in active_chats:
        updater.bot.send_message(chat_id=chat_id, text=ctx["rawio:out"])
