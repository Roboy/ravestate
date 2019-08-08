import random

import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
from unidecode import unidecode
from threading import RLock

from reggol import get_logger
logger = get_logger(__name__)

PYROBOY_AVAILABLE = False
try:
    from ravestate_ros1 import Ros1PubProperty
    from std_msgs.msg import Float32
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

    AXIS0_LOWER_LIMIT_KEY = "head_axis0_lower_limit"
    AXIS0_UPPER_LIMIT_KEY = "head_axis0_upper_limit"
    AXIS1_LOWER_LIMIT_KEY = "head_axis1_lower_limit"
    AXIS1_UPPER_LIMIT_KEY = "head_axis1_upper_limit"
    AXIS2_LOWER_LIMIT_KEY = "head_axis2_lower_limit"
    AXIS2_UPPER_LIMIT_KEY = "head_axis2_upper_limit"
    MOVEMENT_PROBABILITY_KEY = "movement_probability"

    CONFIG = {
        # Directions out of Roboy's perspective
        # Axis 0: >0 lean head right; <0 lean head left
        AXIS0_LOWER_LIMIT_KEY: -0.3,  # lower limit for movement on axis 0
        AXIS0_UPPER_LIMIT_KEY: 0.3,  # upper limit for movement on axis 0

        # Axis 1: >0 lean head backward; <0 lean head forward
        AXIS1_LOWER_LIMIT_KEY: -0.3,  # lower limit for movement on axis 1
        AXIS1_UPPER_LIMIT_KEY: 0.3,  # upper limit for movement on axis 1

        # Axis 2: >0 turn head left; <0 turn head right
        AXIS2_LOWER_LIMIT_KEY: -0.3,  # lower limit for movement on axis 2
        AXIS2_UPPER_LIMIT_KEY: 0.3,  # upper limit for movement on axis 2

        MOVEMENT_PROBABILITY_KEY: 0.5,  # probability for an axis to moves; decided separately for each axis
    }

    with rs.Module(name="roboyio", config=CONFIG):

        neck_axis0_pub_prop = Ros1PubProperty(name="neck_axis0", topic="/neck_axis0/neck_axis0/target", msg_type=Float32)
        neck_axis1_pub_prop = Ros1PubProperty(name="neck_axis1", topic="/neck_axis1/neck_axis1/target", msg_type=Float32)
        neck_axis2_pub_prop = Ros1PubProperty(name="neck_axis2", topic="/neck_axis2/neck_axis2/target", msg_type=Float32)

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

        @rs.state(cond=rawio.prop_in.changed() | idle.sig_bored, write=(neck_axis0_pub_prop, neck_axis1_pub_prop, neck_axis2_pub_prop))
        def move_head(ctx: rs.ContextWrapper):
            for axis, lower, upper in [(neck_axis0_pub_prop, ctx.conf(key=AXIS0_LOWER_LIMIT_KEY), ctx.conf(key=AXIS0_UPPER_LIMIT_KEY)),
                                       (neck_axis1_pub_prop, ctx.conf(key=AXIS1_LOWER_LIMIT_KEY), ctx.conf(key=AXIS1_UPPER_LIMIT_KEY)),
                                       (neck_axis2_pub_prop, ctx.conf(key=AXIS2_LOWER_LIMIT_KEY), ctx.conf(key=AXIS2_UPPER_LIMIT_KEY))]:
                data = random.uniform(lower, upper)
                if random.random() < ctx.conf(key=MOVEMENT_PROBABILITY_KEY):  # move or don't move axis with probability
                    logger.info(f"Publishing {data} to {axis.topic}")
                    ctx[axis] = Float32(data=data)
