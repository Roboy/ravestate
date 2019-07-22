from reggol import get_logger
logger = get_logger(__name__)

from telegram import InlineKeyboardButton, InlineKeyboardMarkup

def start(update, context):
    logger.info("inline keyboard")
    keyboard = [[InlineKeyboardButton("Option 1", callback_data='1'),
                 InlineKeyboardButton("Option 2", callback_data='2')],

                [InlineKeyboardButton("Option 3", callback_data='3')]]

    reply_markup = InlineKeyboardMarkup(keyboard)
    logger.info("reply_makup")
    # update.message.reply_text('Please choose:', reply_markup=reply_markup)
    logger.info('Please choose:')

def button(update, context):
    query = update.callback_query

    query.edit_message_text(text="Selected option: {}".format(query.data))


def help(update, context):
    logger.info("help")
    # update.message.reply_text("Use /start to test this bot.")
    logger.info("Use /start to test this bot.")


def error(update, context):
    """Log Errors caused by Updates."""
    logger.warning('Update "%s" caused error "%s"', update, context.error)
