import os
import multiprocessing as mp
from multiprocessing.connection import Connection
import time
from typing import Set, Dict, Optional, Tuple
from tempfile import mkstemp
import requests

import ravestate as rs

from telegram import Bot, Update, TelegramError
from telegram.ext import Updater, MessageHandler, Filters, Dispatcher, CommandHandler, CallbackQueryHandler
from telegram import InlineKeyboardButton, InlineKeyboardMarkup


# from ravestate_telegramio.inline_keyboard import start, button, help
from scientio.ontology.node import Node
from scientio.session import Session
from scientio.ontology.ontology import Ontology

from reggol import get_logger
logger = get_logger(__name__)

import ravestate_rawio as rawio
import ravestate_interloc as interloc
import ravestate_ontology

import telegram

import random

MODULE_NAME: str = 'telegramio'
TOKEN_CONFIG_KEY: str = "telegram-token"
CHILD_CONN_CONFIG_KEY: str = "child_conn"
CHILD_FILES_CONFIG_KEY: str = "child_config_files"
ALL_IN_ONE_CONTEXT_CONFIG_KEY: str = 'all_in_one_context'
CHAT_LIFETIME: str = 'chat_lifetime'


class Timestamp:
    value: float

    def __init__(self):
        self.value = time.time()

    def update(self):
        self.value = time.time()

    def age(self):
        return time.time() - self.value


# active_chats contains all active "Chats".
# Maps chat_id to a tuple consisting of the timestamp of the last message in this chat and a Pipe.
# If all chats run in one process, the pipe-field is set to None
active_chats: Dict[int, Tuple[Timestamp, Optional[mp.connection.Connection]]] = dict()
# active_users contains user_ids of all Users that currently engage with the bot.
# At the same time a User can talk to the bot in personal and group chats but only is in active_users once.
# A user_id is mapped to a set containing the chat_id of every Chat that the User is involved in
active_users: Dict[int, Set[int]] = dict()

mediapath = "/home/missxa/workspace/ravestate/media"
partygifs = ["avocado.gif", "catrave.gif", "crazygirl.gif", "frog.gif", "glittertoss.gif", "lemon.gif", "parrot.gif", "rave.gif"]

stickers: Dict = {"explosion": "CAADAgADsgAD5dCAEBmMXCCt4Sh6Ag",
                    "dance": "CAADAgADrwAD5dCAELOikjem6GCQAg",
                    "kenny": "CAADAgADkAAD5dCAEGMfygavvZSZAg",
                    "beer": "CAADAgADKQAD5dCAEFX3hCMAAfM_awI",
                    "happy": "CAADAgADRgAD5dCAEJV_o50ekE5HAg",
                    "hearts": "CAADAgADSQAD5dCAEN9n0g-x5va8Ag",
                    "roboyrick": "CAADAgADjwAD5dCAEA26JXGqLEGhAg",
                    "sunglasses": "CAADAgADLwAD5dCAENTFuFLbW8-XAg",
                    "rainbow": "CAADAgADVQAD5dCAEHTBjm9cSbBTAg",
                    "pickle": "CAADAgADwgAD5dCAEKjfQCRuUDfYAg"}


@rs.state(cond=rs.sig_startup)
def telegram_run(ctx: rs.ContextWrapper):
    """
    Starts up the telegram bot and adds a handler to write incoming messages to rawio:in
    """

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
    def text_receptor(ctx: rs.ContextWrapper, message_text: str):
        """
        Writes the message_text to rawio:in
        """
        ctx[rawio.prop_in] = message_text

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_pic_in)
    def photo_receptor(ctx: rs.ContextWrapper, photo_path):
        """
        Handles photo messages, write to rawio:pic_in
        """
        ctx[rawio.prop_pic_in] = photo_path

    @rs.receptor(ctx_wrap=ctx, write=interloc.prop_all)
    def push_telegram_interloc(ctx: rs.ContextWrapper, telegram_node: Node, name: str):
        """
        Push the telegram_node into interloc:all:name
        """
        if ctx.push(parent_property_or_path=interloc.prop_all, child=rs.Property(name=name, default_value=telegram_node)):
            logger.debug(f"Pushed {telegram_node} to interloc:all")

    def make_sure_effective_user_exists(update: Update):
        """
        Retrieves scientio Node of User if it exists, otherwise creates it in the scientio session
        Calls the push_telegram_interloc receptor to push the scientio node into interloc:all
        Adds the User to the set of active_users and the chat to the set of active_chats
        """
        active_chats[update.effective_chat.id] = (Timestamp(), None)
        if update.effective_user.id in active_users:
            active_users[update.effective_user.id].add(update.effective_chat.id)
        else:
            # set up scientio
            if ravestate_ontology.initialized.wait():
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
                active_users[update.effective_user.id] = {update.effective_chat.id}

    def handle_text(bot: Bot, update: Update):
        """
        Handle incoming text messages
        """
        make_sure_effective_user_exists(update)
        text_receptor(update.effective_message.text)

    def handle_photo(bot: Bot, update: Update):
        """
        Handle incoming photo messages.
        """
        make_sure_effective_user_exists(update)
        photo_index = 2  # Seems like a good size index. TODO: Make configurable
        while photo_index >= len(update.effective_message.photo):
            photo_index -= 1
            if photo_index < 0:
                logger.error("Telegram photo handler was called, but no photo received!")
                return
        file_descr = bot.get_file(update.effective_message.photo[photo_index].file_id)
        photo = requests.get(file_descr.file_path)
        file_path = mkstemp()[1]
        with open(file_path, 'wb') as file:
            file.write(photo.content)
        photo_receptor(file_path)

    def handle_input_multiprocess(bot: Bot, update: Update):
        """
        Handle incoming messages
        """
        # logger.info("update:" + update)
        if update.effective_chat.id not in active_chats:
            add_new_child_process(update.effective_chat.id)
        # write (bot, update) to Pipe
        bot.send_chat_action(chat_id=update.effective_chat.id, action=telegram.ChatAction.TYPING)

        active_chats[update.effective_chat.id][0].update()
        active_chats[update.effective_chat.id][1].send((bot, update))

    def add_new_child_process(chat_id):
        """
        Adds the chat of the incoming message to the set of active_chats
        Creates new Ravestate Context in new Process for the new chat and
        sets up a bidirectional Pipe for communication between Master and Child Processes
        """
        # start method has to be 'spawn'
        mp_context = mp.get_context('spawn')
        # Pipe to communicate between Master Process and all children
        parent_conn, child_conn = mp.Pipe()
        # create commandline args for child config file
        args = []
        child_config_paths_list = ctx.conf(key=CHILD_FILES_CONFIG_KEY)
        for child_config_path in child_config_paths_list:
            args += ['-f', child_config_path]
        # set up new Process and override child_conn with the Pipe-Connection
        p = mp_context.Process(target=rs.create_and_run_context,
                               args=(*args,),
                               kwargs={'runtime_overrides': [(MODULE_NAME, CHILD_CONN_CONFIG_KEY, child_conn)]})
        p.start()
        active_chats[chat_id] = (Timestamp(), parent_conn)


    def start(bot: Bot, update: Update):

        with open(mediapath + "/gif/roboyrave.gif", 'rb') as f:
            bot.send_animation(chat_id=update.message.chat_id, animation=f)
        with open(mediapath + "/voice/readytoparty.mp3", 'rb') as f:
            bot.send_voice(chat_id=update.message.chat_id, voice=f)
        # bot.send_sticker(chat_id=update.message.chat_id, sticker="CAADAgADjwAD5dCAEA26JXGqLEGhAg")
        update.message.reply_text("/whoopwhoop")
        # update.message.reply_text('davai davai', reply_markup=reply_markup)

    def button(bot: Bot, update: Update):
        query = update.callback_query
        chat_id = update.callback_query.message.chat_id
        # update.message.reply_text(text="Selected option: {}".format(query.data))
        if query.data == 'location':
            bot.send_location(chat_id=chat_id, latitude=48.263390, longitude=11.668413)
            bot.send_message(chat_id=chat_id, text="be there or be square. Friday 7 pm")
            # bot.send_sticker(chat_id=update.callback_query.message.chat_id, sticker="CAADAgADkAAD5dCAEGMfygavvZSZAg")
        elif query.data == 'drinks':
            m = "i have ordered some beers and prosecco for the rave, so we all have a good time ;) but feel free to bring more booze"
            bot.send_message(chat_id=chat_id, text=m)
            bot.send_sticker(chat_id=chat_id, sticker=random.choice(list(stickers.values())))
        elif query.data == 'gif':
            m = "LETS PARTEEEY!!11!!"
            bot.send_message(chat_id=chat_id, text=m)
            with open(mediapath + "/gif/" + random.choice(partygifs), 'rb') as f:
                bot.send_animation(chat_id=chat_id, animation=f)
        elif query.data == 'tune':
            m = "https://soundcloud.com/mira_kater/mira_garbicz-festival-2018_seebuhne_2018_03_08#t=32:23"
            bot.send_message(chat_id=chat_id, text=m)
        bot.answer_callback_query(update.callback_query.id, text='')

        # query.edit_message_text(text="Selected option: {}".format(query.data))

    def whoopwhoop(bot: Bot, update: Update):
        # rainbow 1F308
        # beer 1F37B
        # martini U+1F378
        # cocktail U+1F379
        # sekt U+1F37E
        # carousel horse 1F3A0
        # dog U+1F415
        # zebra U+1F993
        # octopus U+1F419
        # bomb U+1F4A3
        # pushpin U+1F4CD
        # fire U+1F525
        # sparkles U+2728
        # dizzy face
        # robot U+1F916
        # surfing U+1F3C4

        drinks = InlineKeyboardButton("\U0001F37B \U0001F379 \U0001F37E", callback_data='drinks')
        location = InlineKeyboardButton("\U0001F4CD", callback_data='location')
        gif = InlineKeyboardButton("\U0001F308", callback_data='gif')
        tune = InlineKeyboardButton("\U0001F419", callback_data='tune')

        keyboard = [[location, gif, drinks]]

        reply_markup = InlineKeyboardMarkup(keyboard, one_time_keyboard=True)
        update.message.reply_text('buttons are basic, check them out. i also like to talk', reply_markup=reply_markup)

    def help(bot: Bot, update: Update):
        update.message.reply_text("Use /start to test this bot.")

    def error(bot: Bot, update: Update, error: TelegramError):
        """
        Log Errors caused by Updates.
        """
        logger.warning(f'Update {update.effective_message} caused error {error.message}')

    def _manage_children(updater):
        """
        Receive messages from children via Pipe and then send them to corresponding Telegram Chat.
        Remove chats when they get older than the chat lifetime.
        :param updater: The Updater of the telegram-Bot
        """
        chat_lifetime = ctx.conf(key=CHAT_LIFETIME) * 60  # conversion from minutes to seconds
        while not ctx.shutting_down():
            removable_chats = set()
            removable_users = set()

            # wait for children to write to Pipe and then send message to chat
            tick_interval = 1. / ctx.conf(mod=rs.CORE_MODULE_NAME, key=rs.TICK_RATE_CONFIG_KEY)
            time.sleep(tick_interval)
            for chat_id, (last_msg_timestamp, parent_pipe) in active_chats.items():
                if parent_pipe.poll():
                    msg = parent_pipe.recv()
                    # logger.info("msg: "+ msg)
                    # import pdb; pdb.set_trace()
                    logger.info("type:" + str(type(msg)))
                    logger.info("incoming :" + msg)
                    if isinstance(msg, list):
                        for m in msg:
                            if m.startswith("gif:"):
                                with open(m.split(":")[1], 'rb') as f:
                                    updater.bot.send_animation(chat_id=chat_id, animation=f)
                            elif m.startswith("location:"):
                                latlong = m.split(":")[1]
                                lat = latlong.split(",")[0]
                                long = latlong.split(",")[1]
                                updater.bot.send_location(chat_id=chat_id, latitude=lat, longitude=long)
                            elif m.startswith("sticker:"):
                                updater.bot.send_sticker(chat_id=chat_id, sticker=m.split(":")[1])
                            elif m.startswith("voice:"):
                                with open(m.split(":")[1], 'rb') as f:
                                    updater.bot.send_voice(chat_id=chat_id, voice=f)
                            else:
                                m = m.lower().strip()
                                updater.bot.send_message(chat_id=chat_id, text=m)
                    elif isinstance(msg, str):
                        if msg.startswith("gif:"):
                            with open(msg.split(":")[1], 'rb') as f:
                                updater.bot.send_animation(chat_id=chat_id, animation=f)
                        elif msg.startswith("location:"):
                            latlong = msg.split(":")[1]
                            lat = latlong.split(",")[0]
                            long = latlong.split(",")[1]
                            updater.bot.send_location(chat_id=chat_id, latitude=lat, longitude=long)
                        elif msg.startswith("sticker:"):
                            updater.bot.send_sticker(chat_id=chat_id, sticker=msg.split(":")[1])
                        elif msg.startswith("voice:"):
                            with open(msg.split(":")[1], 'rb') as f:
                                updater.bot.send_voice(chat_id=chat_id, voice=f)
                        else:
                            msg = msg.lower().strip()
                            updater.bot.send_message(chat_id=chat_id, text=msg)
                    else:
                        logger.error(f'Tried sending non-str object as telegram message: {str(msg)}')
                # remove chat from active_chats if inactive for too long
                if last_msg_timestamp.age() > chat_lifetime:
                    parent_pipe.close()
                    removable_chats.add(chat_id)

            for chat_id in removable_chats:
                active_chats.pop(chat_id)
                for user_id, chat_ids in active_users.items():
                    # remove chat from chats that the user is part of
                    chat_ids.discard(chat_id)
                    if len(chat_ids) == 0:
                        # user is no longer part of any active chats
                        removable_users.add(user_id)
            for user_id in removable_users:
                active_users.pop(user_id)

    def _bootstrap_telegram_master():
        """
        Handle TelegramIO as the Master Process.
        Start the bot, and handle incoming telegram messages.
        """
        token = ctx.conf(key=TOKEN_CONFIG_KEY)
        if not token:
            logger.error(f'{TOKEN_CONFIG_KEY} is not set. Shutting down telegramio')
            return rs.Delete()
        child_config_paths_list = ctx.conf(key=CHILD_FILES_CONFIG_KEY)
        if not ctx.conf(key=ALL_IN_ONE_CONTEXT_CONFIG_KEY) and (
                not child_config_paths_list or not isinstance(child_config_paths_list, list)
                or not all(os.path.isfile(child_config_path) for child_config_path in child_config_paths_list)):
            logger.error(f'{CHILD_FILES_CONFIG_KEY} is not set (correctly). Shutting down telegramio')
            return rs.Delete()

        updater: Updater = Updater(token)
        # Get the dispatcher to register handlers
        dispatcher: Dispatcher = updater.dispatcher
        dispatcher.add_handler(CommandHandler('start', start))
        dispatcher.add_handler(CallbackQueryHandler(button))
        dispatcher.add_handler(CommandHandler('help', help))
        dispatcher.add_handler(CommandHandler('whoopwhoop', whoopwhoop))
        if ctx.conf(key=ALL_IN_ONE_CONTEXT_CONFIG_KEY):
            # handle noncommand-messages with the matching handler
            dispatcher.add_handler(MessageHandler(Filters.text, handle_text))
            dispatcher.add_handler(MessageHandler(Filters.photo, handle_photo))
        else:
            dispatcher.add_handler(MessageHandler(Filters.text | Filters.photo , handle_input_multiprocess))

        # log all errors
        dispatcher.add_error_handler(error)
        # Start the Bot
        updater.start_polling()  # non blocking

        if not ctx.conf(key=ALL_IN_ONE_CONTEXT_CONFIG_KEY):
            _manage_children(updater)

    def _bootstrap_telegram_child():
        """
        Handle TelegramIO as a Child Process.
        Listen to Pipe and handle incoming texts and photos.
        """
        try:
            while not ctx.shutting_down():
                # receive Bot,Update for telegram chat
                bot, update = child_conn.recv()  # blocking
                if update.effective_message.photo:
                    handle_photo(bot, update)
                elif update.effective_message.text:
                    handle_text(bot, update)
                else:
                    logger.error(f"{MODULE_NAME} received an update it cannot handle.")
        except EOFError:
            # Pipe was closed -> Parent was killed or parent has closed the pipe
            logger.info("Pipe was closed, therefore the telegram-child will shut down.")
            ctx.shutdown()

    child_conn = ctx.conf(key=CHILD_CONN_CONFIG_KEY)
    is_master_process = child_conn is None
    if is_master_process:
        return _bootstrap_telegram_master()
    else:
        _bootstrap_telegram_child()


@rs.state(read=rawio.prop_out)
def telegram_output(ctx: rs.ContextWrapper):
    """
    If all telegram chats should be in the same context, sends the content of rawio:out to every currently active chat.
    Otherwise it only sends output using the Pipe if it is a child process
    """
    text = ctx[rawio.prop_out.changed()]
    logger.info(text)
    # sticker = ctx[ravebot.prop_sticker.changed()]
    # logger.info("type "+str(type(text)))
    # if not text or not isinstance(text, str) or not isinstance(text, list):
    #     logger.info("resigned")
    #     return rs.Resign()
    # text = text.lower().strip()
    if ctx.conf(key=ALL_IN_ONE_CONTEXT_CONFIG_KEY):
        # TODO don't instantiate the updater every time
        token = ctx.conf(key=TOKEN_CONFIG_KEY)
        if not token:
            logger.error('telegram-token is not set. Shutting down telegramio')
            return rs.Delete()
        updater: Updater = Updater(token)
        for chat_id in active_chats.keys():
            updater.bot.send_message(chat_id=chat_id, text=text)
    else:
        child_conn = ctx.conf(key=CHILD_CONN_CONFIG_KEY)
        if child_conn:
            # Child Process -> write to Pipe
            child_conn.send(text)
        else:
            # Master Process -> State not needed
            return rs.Delete()
