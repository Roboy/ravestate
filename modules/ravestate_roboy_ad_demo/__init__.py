from ravestate.context import startup
from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.wrappers import ContextWrapper
from ravestate.state import state, Resign
from ravestate.constraint import s
from ravestate_nlp.triple import Triple

from ravestate_rawio import output as raw_out
from ravestate_ros2.ros2_properties import Ros2PubProperty, Ros2SubProperty

from std_msgs.msg import String, Bool

from reggol import get_logger
logger = get_logger(__name__)


DR_TO_AD_TOPIC = "/dr_to_ad"
AD_TO_DR_TOPIC = "/ad_to_dr"
SKIN_TO_DR_TOPIC = "/skin_trigger"

PICKUP_REQUESTED_MSG = "pick_up_requested"
START_DRIVING_MSG = "start_driving"

ARRIVED_AT_PICKUP_POINT_MSG = "arrived_at_pick_up_point"
ARRIVED_AT_DROPOFF_POINT_MSG = "arrived_at_drop_off_point"


ROS2_HANDSHAKE_MOTION_TOPIC_CONFIG = "ros2-handshake-motion-topic"
HANDSHAKE_MOTION_NAME_CONFIG = "handshake-motion-name"

CONFIG = {
    ROS2_HANDSHAKE_MOTION_TOPIC_CONFIG: "/roboy_plexus/ReplayTrajectory",  # TODO
    HANDSHAKE_MOTION_NAME_CONFIG: "HandshakeMotion",  # TODO
}

with Module(name="ad_demo", config=CONFIG) as mod:
    # Create a dummy parent, under which we can push the actual recognized faces topic,
    #  once a context with a configuration is available.
    ros2_properties_parent = PropertyBase(name="ros2_parent")


    @state(cond=startup(), write=ros2_properties_parent.id())
    def create_ros2_properties(ctx: ContextWrapper):
        # create publishers and subscribers
        publish_dr_to_ad = Ros2PubProperty(name="publish_dr_to_ad",
                                           topic=DR_TO_AD_TOPIC,
                                           msg_type=String)
        subscribe_ad_to_dr = Ros2SubProperty(name="subscribe_ad_to_dr",
                                             topic=AD_TO_DR_TOPIC,
                                             msg_type=String)
        publish_handshake_motion = Ros2PubProperty(name="publish_handshake_motion",
                                                   topic=ctx.conf(key=ROS2_HANDSHAKE_MOTION_TOPIC_CONFIG),
                                                   msg_type=String)
        subscribe_skin_to_dr = Ros2PubProperty(name="subscribe_skin_to_dr",
                                               topic=SKIN_TO_DR_TOPIC,
                                               msg_type=Bool)

        # add the ros2 properties to context
        ctx.push(ros2_properties_parent.id(), publish_dr_to_ad)
        ctx.push(ros2_properties_parent.id(), subscribe_ad_to_dr)
        ctx.push(ros2_properties_parent.id(), publish_handshake_motion)
        ctx.push(ros2_properties_parent.id(), subscribe_skin_to_dr)

        @state(cond=s("nlp:triples:changed"),
               read="nlp:triples", write=(publish_dr_to_ad.id(), raw_out.id()))
        def pickup_requested(ctx: ContextWrapper):
            # Trigger phrase: "Pick me up."
            triples = ctx["nlp:triples"]
            if triples[0].match_either_lemma(pred={"pick"}):
                ctx[publish_dr_to_ad.id()] = String(data=PICKUP_REQUESTED_MSG)
                ctx[raw_out.id()] = "I'm on my way!"
            else:
                return Resign()

        @state(read=subscribe_ad_to_dr.id(), write=raw_out.id())
        def arrived_somewhere(ctx: ContextWrapper):
            # processes message from AD
            msg = ctx[subscribe_ad_to_dr.id()].data
            if msg == ARRIVED_AT_PICKUP_POINT_MSG:
                ctx[raw_out.id()] = "Hey dad! Hop on!"
            elif msg == ARRIVED_AT_DROPOFF_POINT_MSG:
                ctx[raw_out.id()] = "We have arrived."
            else:
                logger.error(f"Unexpected message: {msg} received on topic: {AD_TO_DR_TOPIC}")

        @state(cond=s("nlp:triples:changed"),
               read="nlp:triples", write=(publish_dr_to_ad.id(), raw_out.id()))
        def start_driving(ctx: ContextWrapper):
            # Trigger phrase: "Start driving." or "Go."
            triples = ctx["nlp:triples"]
            if triples[0].match_either_lemma(pred={"drive", "start", "go"}):
                # lets go
                ctx[publish_dr_to_ad.id()] = String(data=START_DRIVING_MSG)
                ctx[raw_out.id()] = "Fasten your seat belt and off we go!"
            else:
                return Resign()

        @state(read=subscribe_skin_to_dr.id(), write=raw_out.id())
        def handshake_detected(ctx: ContextWrapper):
            msg = ctx[subscribe_skin_to_dr.id()].data
            if msg:
                ctx[raw_out.id()] = "Ah. I see you are very good at handshaking."
            else:
                return Resign()

        @state(cond=s("stalker:lennart"), write=publish_handshake_motion.id())
        def lennart_recognized(ctx: ContextWrapper):
            # check if it is lennart, greets lennart (this is done in stalker) -> TODO signal lennarts-here?
            # triggers handshake: topic ReplayTrajectory which takes a string as a name
            ctx[publish_handshake_motion.id()] = String(data=ctx.conf(key=HANDSHAKE_MOTION_NAME_CONFIG))

        for st in [pickup_requested, start_driving, arrived_somewhere, lennart_recognized, handshake_detected]:
            mod.add(st)
            ctx.add_state(st)
