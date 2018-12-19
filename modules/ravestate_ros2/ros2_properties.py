import time
from typing import Dict, Set

from ravestate.state import state
from ravestate.constraint import s
from ravestate.property import PropertyBase
from threading import Lock, Thread

from ravestate.receptor import receptor
from ravestate.wrappers import ContextWrapper
from reggol import get_logger
logger = get_logger(__name__)

ROS2_AVAILABLE = True  # TODO check for this in the constructors and make properties without ros2
try:
    import rclpy
except ImportError:
    logger.error("Could not import rclpy. Please make sure to have ROS2 installed.")
    ROS2_AVAILABLE = False


subprop_to_receptor = dict()
receptor_ctx = None

rclpy.init()
node = rclpy.create_node("ros2_ravestate")  # TODO nodename in config
node_lock = Lock()


@state(triggers=s(":startup"))
def register_ros_subscribers(ctx: ContextWrapper):
    global node, node_lock, subprop_to_receptor
    current_props: Set = set()
    while not ctx.shutting_down():
        # remove deleted props
        removed_props = current_props - subprop_to_receptor.keys()
        for prop in removed_props:
            if isinstance(prop, Ros2SubProperty):
                node.destroy_subscription(prop.subscription)
            elif isinstance(prop, Ros2PubProperty):
                node.destroy_publisher(prop.publisher)
            current_props.remove(prop)

        # add new props
        new_props = subprop_to_receptor.keys() - current_props
        for prop in new_props:
            # register in context
            @receptor(ctx_wrap=ctx, write=prop.fullname())
            def ros_subscription_callback(ctx, msg, prop_name: str):
                ctx[prop_name] = msg

            subprop_to_receptor[prop] = ros_subscription_callback
            # register subscribers in ROS
            if isinstance(prop, Ros2SubProperty):
                prop.subscription = node.create_subscription(prop.msg_type, prop.topic, prop.subscriber_callback)
            # save in current props
            current_props.add(prop)

        # spin once
        with node_lock:
            rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.1)  # TODO sleep or not?


# TODO destroy node?

class Ros2SubProperty(PropertyBase):
    # TODO use kwargs for super params?
    def __init__(self, name: str, topic: str, msg_type):  # TODO annotate rosmsg-type
        super().__init__(name=name)
        self.topic = topic
        self.msg_type = msg_type
        global subprop_to_receptor
        subprop_to_receptor[self] = None

    def __del__(self):
        global subprop_to_receptor
        subprop_to_receptor.pop(self)

    def subscriber_callback(self, msg):
        """
        Writes the message data to the property
        """
        global subprop_to_receptor
        # only after :startup
        if subprop_to_receptor[self]:
            logger.debug(f"Reveived message {msg.data} from topic {self.topic}")
            subprop_to_receptor[self](msg=msg.data, prop_name=self.fullname())


class Ros2PubProperty(PropertyBase):
    def __init__(self, name: str, topic: str, msg_type):  # TODO annotate rosmsg-type
        super().__init__(name=name)
        self.topic = topic
        self.msg_type = msg_type
        global node, node_lock
        with node_lock:
            self.publisher = node.create_publisher(self.msg_type, self.topic)

    def write(self, value):
        """
        Publish value on ROS
        :param value: ROS-Message that should be published
        """
        if super().write(value):
            logger.debug(f"Publishing message {value.data} on topic {self.topic}")
            if self.publisher:
                self.publisher.publish(value)
