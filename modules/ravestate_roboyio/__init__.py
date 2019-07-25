import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
from unidecode import unidecode
from threading import RLock
import time

from reggol import get_logger
logger = get_logger(__name__)
import pdb

PYROBOY_AVAILABLE = False
try:
    from pyroboy import say, listen
    from roboy_cognition_msgs.msg import RecognizedSpeech
    from roboy_cognition_msgs.srv import RecognizeSpeech
    PYROBOY_AVAILABLE = True
except ImportError as e:
    logger.error(f"""
--------
An exception occured during imports: {e}
Please make sure to have the following items installed & sourced:
1. ROS2 bouncy
2. roboy_communication
3. pyroboy
--------
    """)


if PYROBOY_AVAILABLE:

    say_lock = RLock()
    listen_start = 0
    global say_end
    say_end = 0

    with rs.Module(name="roboyio"):

        @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
        def roboy_input(ctx: rs.ContextWrapper):
            while not ctx.shutting_down():
                #with say_lock:
                    #pdb.set_trace()
                listen_start = time.time()
                result = listen()
                logger.info(f"pyroboy.listen() -> {result}")
                global say_end
                logger.info(f"listen_start: {listen_start}; say_end: {say_end}")
                if result and listen_start > say_end:
                    interloc.handle_single_interlocutor_input(ctx, result)
                else:
                    logger.info(f"Discarded {result}")
                #time.sleep(1)  # LEDs are indicator of when roboy is listening

        @rs.state(read=rawio.prop_out)
        def roboy_output(ctx):
            # don't call say simultaneously in different threads
            with say_lock:
                ret = say(unidecode(ctx[rawio.prop_out.changed()]))
                global say_end
                say_end = time.time()
            logger.info(f"pyroboy.say({ctx[rawio.prop_out.changed()]}) -> {ret}")
