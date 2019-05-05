from ravestate.module import Module
from ravestate.state import state
from ravestate.wrappers import ContextWrapper
from ravestate_interloc import handle_single_interlocutor_input
from ravestate_ros2.ros2_properties import Ros2SubProperty
from ravestate.context import startup
import ravestate_ros2
from unidecode import unidecode
from reggol import get_logger
logger = get_logger(__name__)

from time import sleep

PYROBOY_AVAILABLE = False
try:
    from pyroboy import say, listen, node
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

    ravestate_ros2.set_node_once(node)

    with Module(name="roboyio"):

        recognized_speech = Ros2SubProperty(
            name="recognized_speech",
            topic="/roboy/cognition/speech/recognition",
            msg_type=RecognizedSpeech,
            always_signal_changed=True)

        @state(cond=recognized_speech.changed_signal(), read=("interloc:all", recognized_speech.id()))
        def roboy_input(ctx: ContextWrapper):
            result = ctx[recognized_speech.id()]
            if result:
                handle_single_interlocutor_input(ctx, result.text)

        # @state(cond=startup(), read="interloc:all")
        # def roboy_input(ctx: ContextWrapper):
        #     while not ctx.shutting_down():
        #         result = listen()
        #         if result:
        #             handle_single_interlocutor_input(ctx, result)

        @state(read="rawio:out")
        def roboy_output(ctx):
            ret = say(unidecode(ctx["rawio:out:changed"]))
            logger.info(f"pyroboy.say() -> {ret}")
