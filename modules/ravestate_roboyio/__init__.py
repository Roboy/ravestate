from ravestate.module import Module
from ravestate.constraint import s
from ravestate.state import state
from ravestate.wrappers import ContextWrapper
from ravestate_interloc import handle_single_interlocutor_input
from ravestate_ros2.ros2_properties import Ros2SubProperty

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
1. ROS2 bouncy
2. roboy_communication
3. pyroboy
--------
    """)


if PYROBOY_AVAILABLE:

    with Module(name="roboyio"):

        @state(cond=s(":startup"), read="interloc:all")
        def roboy_input(ctx: ContextWrapper):
            while not ctx.shutting_down():
                input_value = listen()
                if input_value and input_value.strip():
                    handle_single_interlocutor_input(ctx, input_value)

        @state(read="rawio:out")
        def roboy_output(ctx):
            say(ctx["rawio:out:changed"])
