from ravestate.context import startup
from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.wrappers import ContextWrapper
from ravestate.state import state, Resign
from ravestate.constraint import s
from ravestate_nlp.triple import Triple

from ravestate_ros2.ros2_properties import Ros2PubProperty, Ros2SubProperty

from std_msgs.msg import String

from reggol import get_logger
logger = get_logger(__name__)

ROS2_PICKUP_REQUESTED_TOPIC_CONFIG = "ros2-pickup-requested-topic"

CONFIG = {
    ROS2_PICKUP_REQUESTED_TOPIC_CONFIG: "/ros2_pickup_requested", # TODO
}

with Module(name="ad_demo", config=CONFIG) as mod:
    # Create a dummy parent, under which we can push the actual recognized faces topic,
    #  once a context with a configuration is available.
    subscriber_parent = PropertyBase(name="ros2_parent")


    @state(cond=startup(), write=subscriber_parent.id())
    def create_ros2_properties(ctx: ContextWrapper):
        publish_pickup_requested = Ros2PubProperty(name="publish_pickup_requested",
                                                   topic=ctx.conf(key=ROS2_PICKUP_REQUESTED_TOPIC_CONFIG),
                                                   msg_type=String)

        ctx.push(subscriber_parent.id(), publish_pickup_requested)

        @state(cond=s("nlp:triples:changed"),
               read="nlp:triples", write=publish_pickup_requested.id())
        def pickup_requested(ctx: ContextWrapper):
            # Trigger phrase: "Pick me up."
            triples = ctx["nlp:triples"]
            if triples[0].match_either_lemma(pred={"pick"}):
                ctx[publish_pickup_requested.id()] = String(data="Pick me up!")
            else:
                return Resign()


        @state()
        def arrived_at_pickup_point(ctx: ContextWrapper):
            # processes signal from AD
            # voices: "Hop on!"
            pass

        @state(cond=s("nlp:triples:changed"),
               read="nlp:triples")
        def start_driving(ctx: ContextWrapper):
            # Trigger phrase: "Start driving." or "Go."
            triples = ctx["nlp:triples"]
            if triples[0].match_either_lemma(pred={"drive", "start", "go"}):
                # Msg lets go
                pass
            else:
                return Resign()

        @state()
        def arrived_at_drop_off_point(ctx: ContextWrapper):
            # processes signal from AD
            # Voices. "We have arrived"
            pass


        @state(cond=s("stalker:recognized_face:changed"))
        def lennart_recognized(ctx: ContextWrapper):
            # check if it is lennart
            # greets lennart
            # triggers handshake: topic ReplayTrajectory which takes a string as a name
            pass

        for st in [pickup_requested]:  #, arrived_at_pickup_point, arrived_at_drop_off_point, lennart_recognized]:
            mod.add(st)
            ctx.add_state(st)
