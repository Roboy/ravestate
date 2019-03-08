from ravestate.module import Module
from ravestate.constraint import s
from ravestate.state import state
from ravestate.wrappers import ContextWrapper
from ravestate_interloc import handle_single_interlocutor_input
from ravestate_ros2.ros2_properties import Ros2SubProperty, Ros2CallProperty
from ravestate.context import startup
from threading import Semaphore

from reggol import get_logger
logger = get_logger(__name__)

from time import sleep

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

    with Module(name="roboyio"):

        recognized_speech = Ros2SubProperty(
            name="recognized_speech",
            topic="/roboy/cognition/speech/recognition",
            msg_type=RecognizedSpeech,
            always_signal_changed=True)

        #service_recognized_speech = Ros2CallProperty(name="service_recognized_speech",
        #                                             service_name="/roboy/cognition/speech/recognition",
        #                                             service_type=RecognizeSpeech,
        #                                             call_timeout=10.0)

        @state(cond=startup(), read="interloc:all") #, read=service_recognized_speech.id(), write=service_recognized_speech.id())
        def roboy_input(ctx: ContextWrapper):
            while not ctx.shutting_down():
                #ctx[service_recognized_speech.id()] = RecognizeSpeech.Request()
                #result = ctx[service_recognized_speech.id()]
                result = listen()
                if result:  # TODO what is returned when nothing understood?
                    handle_single_interlocutor_input(ctx, result)


        @state(read="rawio:out")
        def roboy_output(ctx):
            ret = say(ctx["rawio:out:changed"])
            logger.info(f"pyroboy.say() -> {ret}")
