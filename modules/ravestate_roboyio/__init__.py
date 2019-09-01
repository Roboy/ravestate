import random

import os

import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
import ravestate_emotion as emotion
from unidecode import unidecode
from threading import RLock

from reggol import get_logger
logger = get_logger(__name__)

PYROBOY_AVAILABLE = False
try:
    from ravestate_ros1 import Ros1PubProperty, Ros1SubProperty
    from std_msgs.msg import Float32, String
    from roboy_cognition_msgs.msg import RecognizedSpeech
    from pyroboy import say, listen, leds
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
import time
global listen_start_timestamp
global say_end_timestamp
listen_start_timestamp = 0
say_end_timestamp = 0

if PYROBOY_AVAILABLE:

    say_lock = RLock()

    AXIS0_LOWER_LIMIT_KEY = "head_axis0_lower_limit"
    AXIS0_UPPER_LIMIT_KEY = "head_axis0_upper_limit"
    AXIS1_LOWER_LIMIT_KEY = "head_axis1_lower_limit"
    AXIS1_UPPER_LIMIT_KEY = "head_axis1_upper_limit"
    AXIS2_LOWER_LIMIT_KEY = "head_axis2_lower_limit"
    AXIS2_UPPER_LIMIT_KEY = "head_axis2_upper_limit"
    HEAD_MOVEMENT_PROBABILITY_KEY = "head_movement_probability"
    EYE_MOVEMENT_PROBABILITY_KEY = "eye_movement_probability"

    CONFIG = {
        # Directions out of Roboy's perspective
        # Axis 0: <0 lean head backwards; >0 lean head forwards
        AXIS0_LOWER_LIMIT_KEY: -0.3,  # lower limit for movement on axis 0
        AXIS0_UPPER_LIMIT_KEY: 0.3,  # upper limit for movement on axis 0

        # Axis 1: <0 lean head right; >0 lean head left
        AXIS1_LOWER_LIMIT_KEY: -0.3,  # lower limit for movement on axis 1
        AXIS1_UPPER_LIMIT_KEY: 0.3,  # upper limit for movement on axis 1

        # Axis 2: <0 turn head right; >0 turn head left
        AXIS2_LOWER_LIMIT_KEY: -0.3,  # lower limit for movement on axis 2
        AXIS2_UPPER_LIMIT_KEY: 0.3,  # upper limit for movement on axis 2

        HEAD_MOVEMENT_PROBABILITY_KEY: 0.5,  # probability for an axis to moves; decided separately for each axis
        EYE_MOVEMENT_PROBABILITY_KEY: 0.5,  # probability for eyes to look left/right/
    }

    LOOK_LEFT_EMOTION = "lookleft"
    LOOK_RIGHT_EMOTION = "lookright"
    SHY_EMOTION = "shy"
    SURPRISE_EMOTION = "surprise"
    SMILEBLINK_EMOTION = "smileblink"
    LUCKY_EMOTION = "lucky"
    KISS_EMOTION = "kiss"




    with rs.Module(name="roboyio", config=CONFIG):

        prop_head_axis0 = Ros1PubProperty(name="head_axis0", topic="/sphere_head_axis0/sphere_head_axis0/target", msg_type=Float32)
        prop_head_axis1 = Ros1PubProperty(name="head_axis1", topic="/sphere_head_axis1/sphere_head_axis1/target", msg_type=Float32)
        prop_head_axis2 = Ros1PubProperty(name="head_axis2", topic="/sphere_head_axis2/sphere_head_axis2/target", msg_type=Float32)

        prop_show_emotion = Ros1PubProperty(name="show_emotion", topic="/roboy/cognition/face/show_emotion", msg_type=String)
        prop_move_eyes = Ros1PubProperty(name="move_eyes", topic="/roboy/cognition/face/show_emotion", msg_type=String)

        recognized_speech = Ros1SubProperty(
            name="recognized_speech",
            topic="/roboy/cognition/speech/recognition",
            msg_type=RecognizedSpeech,
            always_signal_changed=True)

        @rs.state(cond=recognized_speech.changed(), read=(interloc.prop_all,recognized_speech.id()) )
        def roboy_input(ctx: rs.ContextWrapper):

            # global say_end_timestamp
            logger.info("in roboy_input again")
            result = ctx[recognized_speech.id()]
            # listen_start_timestamp = result.start_timestamp
            # if say_end_timestamp >= listen_start_timestamp:
                # logger.info("Discarded: " + str(result.text))
                # result = None
                # interloc.handle_single_interlocutor_input(ctx, "")
            # else:
            if result.text:
                    logger.info(result.text)
                    interloc.handle_single_interlocutor_input(ctx, result.text)

            # while not ctx.shutting_down():
            #     listen_start_timestamp = time.time()
            #     result = result = ctx[recognized_speech.id()]
            #     print(result)
            #     logger.info(f"pyroboy.listen() -> {result}")
            #     if say_end_timestamp >= listen_start_timestamp:
            #         logger.info("Discarded: " + str(result))
            #         result = None
            #         interloc.handle_single_interlocutor_input(ctx, "")
            #     else:
            #         if result:
            #             interloc.handle_single_interlocutor_input(ctx, result)

        @rs.state(read=rawio.prop_out)
        def roboy_output(ctx):
            # don't call say simultaneously in different threads
            global say_end_timestamp
            try:
                os.system('/raveskills/off.sh')
            except:
                logger.info("does /raveskills/off.sh exist?")
            with say_lock:
                # logger.warn(unidecode(ctx[rawio.prop_out.changed()]))
                ret = say(unidecode(ctx[rawio.prop_out.changed()]))
            say_end_timestamp = time.time()
            try:
                os.system('/raveskills/rainbow.sh')
            except:
                logger.info("does /raveskills/rainbow.sh exist?")
            # logger.info(f"pyroboy.say({ctx[rawio.prop_out.changed()]}) -> {ret}")\


        @rs.state(cond=rawio.prop_in.changed() | idle.sig_bored, write=(prop_head_axis0, prop_head_axis1, prop_head_axis2))
        def move_head(ctx: rs.ContextWrapper):
            for axis, lower, upper in [(prop_head_axis0, ctx.conf(key=AXIS0_LOWER_LIMIT_KEY), ctx.conf(key=AXIS0_UPPER_LIMIT_KEY)),
                                       (prop_head_axis1, ctx.conf(key=AXIS1_LOWER_LIMIT_KEY), ctx.conf(key=AXIS1_UPPER_LIMIT_KEY)),
                                       (prop_head_axis2, ctx.conf(key=AXIS2_LOWER_LIMIT_KEY), ctx.conf(key=AXIS2_UPPER_LIMIT_KEY))]:
                data = random.uniform(lower, upper)
                if random.random() < ctx.conf(key=HEAD_MOVEMENT_PROBABILITY_KEY):  # move or don't move axis with probability
                    logger.info(f"Publishing {data} to {axis.topic}")
                    ctx[axis] = Float32(data=data)

        @rs.state(cond=rawio.prop_in.changed() | idle.sig_bored, write=prop_move_eyes)
        def move_eyes(ctx: rs.ContextWrapper):
            if random.random() < ctx.conf(key=EYE_MOVEMENT_PROBABILITY_KEY):
                if random.random() < 0.5:
                    logger.info(f"Publishing {LOOK_LEFT_EMOTION} to {prop_move_eyes.topic}")
                    ctx[prop_move_eyes] = String(data=LOOK_LEFT_EMOTION)
                else:
                    logger.info(f"Publishing {LOOK_RIGHT_EMOTION} to {prop_move_eyes.topic}")
                    ctx[prop_move_eyes] = String(data=LOOK_RIGHT_EMOTION)

        @rs.state(cond=emotion.sig_shy, write=prop_show_emotion)
        def show_shy(ctx: rs.ContextWrapper):
            logger.info(f"Publishing {SHY_EMOTION} to {prop_show_emotion.topic}")
            ctx[prop_show_emotion] = String(data=SHY_EMOTION)

        @rs.state(cond=emotion.sig_surprise, write=prop_show_emotion)
        def show_surprise(ctx: rs.ContextWrapper):
            logger.info(f"Publishing {SURPRISE_EMOTION} to {prop_show_emotion.topic}")
            ctx[prop_show_emotion] = String(data=SURPRISE_EMOTION)

        @rs.state(cond=emotion.sig_happy, write=prop_show_emotion)
        def show_happy(ctx: rs.ContextWrapper):
            if random.random() < 0.5:
                happy_emotion = SMILEBLINK_EMOTION
            else:
                happy_emotion = LUCKY_EMOTION
            logger.info(f"Publishing {happy_emotion} to {prop_show_emotion.topic}")
            ctx[prop_show_emotion] = String(data=happy_emotion)

        @rs.state(cond=emotion.sig_affectionate, write=prop_show_emotion)
        def show_affectionate(ctx: rs.ContextWrapper):
            if random.random() < 0.5:
                affectionate_emotion = KISS_EMOTION
            else:
                affectionate_emotion = LUCKY_EMOTION
            logger.info(f"Publishing {affectionate_emotion} to {prop_show_emotion.topic}")
            ctx[prop_show_emotion] = String(data=affectionate_emotion)
