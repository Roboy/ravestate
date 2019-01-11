from typing import Set

from ravestate.constraint import s
from ravestate.property import PropertyBase
from ravestate.receptor import receptor
from ravestate.state import state, Delete
from ravestate.wrappers import ContextWrapper
from telegram import Bot, Update, TelegramError
from telegram.ext import Updater, MessageHandler, Filters, Dispatcher

from scientio.ontology.node import Node
from scientio.session import Session
from scientio.ontology.ontology import Ontology

from reggol import get_logger
logger = get_logger(__name__)

import ravestate_rawio
import ravestate_interloc
import ravestate_ontology


TOKEN_CONFIG_KEY: str = "telegram-token"
active_chats: Set[int] = set()
active_users: Set[int] = set()


@state(cond=s(":startup"))
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

    @receptor(ctx_wrap=ctx, write="interloc:all")
    def push_telegram_interloc(ctx: ContextWrapper, telegram_node: Node, name: str):
        """
        Push the telegram_node into interloc:all:name
        """
        if ctx.push(parentpath="interloc:all", child=PropertyBase(name=name, default_value=telegram_node)):
            logger.debug(f"Pushed {telegram_node} to interloc:all")

    def handle_input(bot: Bot, update: Update):
        """
        Handler for incoming messages
        Adds the chat/user of the incoming message to the set of active_chats/active_users
        Calls the telegram_callback receptor to process the incoming message
        Retrieves scientio Node of User if it exists, otherwise creates it in the scientio session
        Calls the push_telegram_interloc receptor to push the scientio node into interloc:all
        """
        telegram_callback(update.effective_message.text)
        active_chats.add(update.effective_chat.id)

        if update.effective_user not in active_users:
            # set up scientio
            sess: Session = ravestate_ontology.get_session()
            onto: Ontology = ravestate_ontology.get_ontology()

            # create scientio Node of type TelegramPerson
            query = Node(metatype=onto.get_type("TelegramPerson"))
            prop_dict = {'telegram_id': update.effective_user.id}
            if update.effective_user.username:
                prop_dict['name'] = update.effective_user.username
            if update.effective_user.full_name:
                prop_dict['full_name'] = update.effective_user.full_name
            query.set_properties(prop_dict)

            node_list = sess.retrieve(query)
            if not node_list:
                telegram_node = sess.create(query)
                logger.info(f"Created new Node in scientio session: {telegram_node}")
            elif len(node_list) == 1:
                telegram_node = node_list[0]
            else:
                logger.error(f'Found multiple TelegramPersons that matched query: {update.message.chat_id} '
                             f'in scientio session. Cannot push node to interloc:all!')
                return

            # push chat-Node
            push_telegram_interloc(telegram_node, update.effective_chat.id)
            active_users.add(update.effective_user)

    def error(bot: Bot, update: Update, error: TelegramError):
        """
        Log Errors caused by Updates.
        """
        logger.warning('Update "%s" caused error "%s"', update, error)

    """Start the bot."""
    # Create the EventHandler and pass it your bots token.
    token = ctx.conf(key=TOKEN_CONFIG_KEY)
    if not token:
        logger.error('telegram-token is not set. Shutting down telegramio')
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
        logger.error('telegram-token is not set. Shutting down telegramio')
        return Delete()

    updater: Updater = Updater(token)
    for chat_id in active_chats:
        updater.bot.send_message(chat_id=chat_id, text=ctx["rawio:out"])
