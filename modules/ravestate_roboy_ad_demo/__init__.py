from ravestate.context import startup
from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.wrappers import ContextWrapper
from ravestate.state import state, Resign
from ravestate.constraint import s
from ravestate_nlp.triple import Triple

from ravestate_rawio import output as raw_out
from ravestate_ros2.ros2_properties import Ros2PubProperty, Ros2SubProperty

from std_msgs.msg import String

from reggol import get_logger
logger = get_logger(__name__)

ROS2_PICKUP_REQUESTED_TOPIC_CONFIG = "ros2-pickup-requested-topic"
ROS2_START_DRIVING_TOPIC_CONFIG = "ros2-start-driving-topic"
ROS2_ARRIVED_AT_PICKUP_TOPIC_CONFIG = "ros2-arrived-at-pickup-topic"
ROS2_ARRIVED_AT_DROPOFF_TOPIC_CONFIG = "ros2-arrived-at-dropoff-topic"

CONFIG = {
    ROS2_PICKUP_REQUESTED_TOPIC_CONFIG: "/ros2_pickup_requested", # TODO
    ROS2_START_DRIVING_TOPIC_CONFIG: "/ros2_start_driving", # TODO
    ROS2_ARRIVED_AT_PICKUP_TOPIC_CONFIG: "/ros2_arrived_at_pickup", # TODO
    ROS2_ARRIVED_AT_DROPOFF_TOPIC_CONFIG: "/ros2_arrived_at_dropoff", # TODO
}

with Module(name="ad_demo", config=CONFIG) as mod:
    # Create a dummy parent, under which we can push the actual recognized faces topic,
    #  once a context with a configuration is available.
    ros2_properties_parent = PropertyBase(name="ros2_parent")


    @state(cond=startup(), write=ros2_properties_parent.id())
    def create_ros2_properties(ctx: ContextWrapper):
        # create publishers and subscribers
        publish_pickup_requested = Ros2PubProperty(name="publish_pickup_requested",
                                                   topic=ctx.conf(key=ROS2_PICKUP_REQUESTED_TOPIC_CONFIG),
                                                   msg_type=String)
        publish_start_driving = Ros2PubProperty(name="publish_start_driving",
                                                topic=ctx.conf(key=ROS2_START_DRIVING_TOPIC_CONFIG),
                                                msg_type=String)
        subscribe_arrived_at_pickup_point = Ros2SubProperty(name="subscribe_arrived_at_pickup_point",
                                                            topic=ctx.conf(key=ROS2_ARRIVED_AT_PICKUP_TOPIC_CONFIG),
                                                            msg_type=String)
        subscribe_arrived_at_dropoff_point = Ros2SubProperty(name="subscribe_arrived_at_dropoff_point",
                                                             topic=ctx.conf(key=ROS2_ARRIVED_AT_DROPOFF_TOPIC_CONFIG),
                                                             msg_type=String)

        # add the ros2 properties to context
        ctx.push(ros2_properties_parent.id(), publish_pickup_requested)
        ctx.push(ros2_properties_parent.id(), publish_start_driving)
        ctx.push(ros2_properties_parent.id(), subscribe_arrived_at_pickup_point)
        ctx.push(ros2_properties_parent.id(), subscribe_arrived_at_dropoff_point)

        @state(cond=s("nlp:triples:changed"),
               read="nlp:triples", write=publish_pickup_requested.id())
        def pickup_requested(ctx: ContextWrapper):
            # Trigger phrase: "Pick me up."
            triples = ctx["nlp:triples"]
            if triples[0].match_either_lemma(pred={"pick"}):
                ctx[publish_pickup_requested.id()] = String(data="Pick me up")  # TODO
            else:
                return Resign()

        @state(read=subscribe_arrived_at_pickup_point.id(), write=raw_out.id())
        def arrived_at_pickup_point(ctx: ContextWrapper):
            # processes signal from AD
            # voices: "Hop on!"
            ctx[raw_out.id()] = "Hop on!"

        @state(cond=s("nlp:triples:changed"),
               read="nlp:triples", write=publish_start_driving.id())
        def start_driving(ctx: ContextWrapper):
            # Trigger phrase: "Start driving." or "Go."
            triples = ctx["nlp:triples"]
            if triples[0].match_either_lemma(pred={"drive", "start", "go"}):
                # lets go
                ctx[publish_start_driving.id()] = String(data="lets go")  # TODO
            else:
                return Resign()

        @state(read=subscribe_arrived_at_dropoff_point.id(), write=raw_out.id())
        def arrived_at_drop_off_point(ctx: ContextWrapper):
            # processes signal from AD
            # Voices. "We have arrived"
            ctx[raw_out.id()] = "We have arrived."

        @state(cond=s("stalker:recognized_face:changed"))
        def lennart_recognized(ctx: ContextWrapper):
            # check if it is lennart
            # greets lennart
            # triggers handshake: topic ReplayTrajectory which takes a string as a name
            pass

        for st in [pickup_requested, arrived_at_pickup_point, arrived_at_drop_off_point]:  # , lennart_recognized]:
            mod.add(st)
            ctx.add_state(st)
