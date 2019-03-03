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
    from pyroboy import say
    from roboy_cognition_msgs.msg import RecognizedSpeech
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

        recognized_speech = Ros2SubProperty(
            name="recognized_speech",
            topic="/roboy/cognition/speech/recognition",
            msg_type=RecognizedSpeech,
            always_signal_changed=True)

        @state(read=recognized_speech.id())
        def roboy_input(ctx: ContextWrapper):
            handle_single_interlocutor_input(ctx, ctx[recognized_speech.id()].text)

        @state(read="rawio:out")
        def roboy_output(ctx):
            ret = say(ctx["rawio:out:changed"])
            logger.info(str(ret))
