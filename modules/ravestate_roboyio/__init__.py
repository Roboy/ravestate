import ravestate as rs
import ravestate_interloc as interloc
import ravestate_ros2 as ros2
import ravestate_rawio as rawio
from unidecode import unidecode

from reggol import get_logger
logger = get_logger(__name__)


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

    ros2.set_node_once(node)

    with rs.Module(name="roboyio"):

        recognized_speech = ros2.Ros2SubProperty(
            name="recognized_speech",
            topic="/roboy/cognition/speech/recognition",
            msg_type=RecognizedSpeech,
            always_signal_changed=True)

        @rs.state(cond=recognized_speech.changed(), read=(interloc.prop_all, recognized_speech))
        def roboy_input(ctx: rs.ContextWrapper):
            result = ctx[recognized_speech]
            if result:
                interloc.handle_single_interlocutor_input(ctx, result.text)

        # @rs.state(cond=startup(), read=interloc.prop_all)
        # def roboy_input(ctx: ContextWrapper):
        #     while not ctx.shutting_down():
        #         result = listen()
        #         if result:
        #             interloc.handle_single_interlocutor_input(ctx, result)

        @rs.state(read=rawio.prop_out)
        def roboy_output(ctx):
            ret = say(unidecode(ctx[rawio.prop_out.changed()]))
            logger.info(f"pyroboy.say() -> {ret}")
