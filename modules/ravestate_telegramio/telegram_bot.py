import os
from typing import Set

from ravestate.context import Context
from ravestate.receptor import receptor
from ravestate.state import state
from ravestate.wrappers import ContextWrapper
from telegram import Bot, Update, TelegramError
from telegram.ext import Updater, MessageHandler, Filters, Dispatcher
import logging
from yaml import load


def get_bot_token(path):
    """
    Get the telegram_bot_token from the API.key file
    :param path: the path to the API.key file
    :return: None if there was an error getting the token, otherwise the telegram_bot_token
    """
    try:
        with open(path, 'r') as keyfile:
            keydict = load(keyfile)
    except FileNotFoundError:
        logging.error('API.key for telegram_bot_token not found')
        return None
    try:
        return keydict['telegram_bot_token']
    except KeyError:
        logging.error('No telegram_bot_token in the API.key file found')
        return None


TELEGRAM_BOT_TOKEN: str = get_bot_token(
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), "resources", "API.key"))
active_chats: Set[int] = set()


@state(triggers=":startup")
def telegram_run(ctx: Context):
    """
    Starts up the telegram bot and adds a handler to write incoming messages to rawio:in
    """

    @receptor(ctx_wrap=ctx, write="rawio:in")
    def telegram_callback(ctx: Context, message_text: str):
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
    # Create the EventHandler and pass it your bot's token.
    if not TELEGRAM_BOT_TOKEN:
        logging.error('TELEGRAM_BOT_TOKEN is not set. Shutting down telegramio')
        return ContextWrapper.DeleteMe

    updater: Updater = Updater(TELEGRAM_BOT_TOKEN)
    # Get the dispatcher to register handlers
    dp: Dispatcher = updater.dispatcher
    # handle noncommand-messages with handle_input
    dp.add_handler(MessageHandler(Filters.text, handle_input))
    # log all errors
    dp.add_error_handler(error)
    # Start the Bot
    updater.start_polling()


@state(read="rawio:out")
def telegram_output(ctx: Context):
    """
    Sends the content of rawio:out to every currently active chat
    """
    # TODO don't instantiate the updater every time
    if not TELEGRAM_BOT_TOKEN:
        logging.error('TELEGRAM_BOT_TOKEN is not set. Shutting down telegramio')
        return ContextWrapper.DeleteMe

    updater: Updater = Updater(TELEGRAM_BOT_TOKEN)
    for chat_id in active_chats:
        updater.bot.send_message(chat_id=chat_id, text=ctx["rawio:out"])
