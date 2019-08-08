import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
from unidecode import unidecode
from threading import RLock

from reggol import get_logger
logger = get_logger(__name__)

PYROBOY_AVAILABLE = False
try:
    from pyroboy import say, listen
    PYROBOY_AVAILABLE = True
except ImportError as e:
    logger.error(f"""
--------
An exception occured during imports: {e}
Please make sure to have the following items installed & sourced:
1. ROS melodic
2. roboy_communication
3. pyroboy
--------
    """)


if PYROBOY_AVAILABLE:

    say_lock = RLock()

    with rs.Module(name="roboyio"):

        @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
        def roboy_input(ctx: rs.ContextWrapper):
            while not ctx.shutting_down():
                result = listen()
                logger.info(f"pyroboy.listen() -> {result}")
                interloc.handle_single_interlocutor_input(ctx, result)

        @rs.state(read=rawio.prop_out)
        def roboy_output(ctx):
            # don't call say simultaneously in different threads
            with say_lock:
                ret = say(unidecode(ctx[rawio.prop_out.changed()]))
            logger.info(f"pyroboy.say({ctx[rawio.prop_out.changed()]}) -> {ret}")
