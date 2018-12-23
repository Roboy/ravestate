import time
from typing import Set

from ravestate.state import state, Delete
from ravestate.constraint import s
from ravestate.property import PropertyBase

from ravestate.receptor import receptor
from ravestate.wrappers import ContextWrapper

from reggol import get_logger
logger = get_logger(__name__)

ROS2_AVAILABLE = True
try:
    import rclpy
except ImportError:
    logger.error("Could not import rclpy. Please make sure to have ROS2 installed.")
    ROS2_AVAILABLE = False

NODE_NAME_CONFIG_KEY = "ros2-node-name"
SPIN_FREQUENCY_CONFIG_KEY = "ros2-spin-frequency"

global_prop_set = set()


@state(triggers=s(":startup"))
def sync_ros_properties(ctx: ContextWrapper):
    """
    State that creates a ROS2-Node, registers all Ros2SubProperties and Ros2PubProperties in ROS2 and keeps them synced
    """
    # check for ROS2 availability
    if not ROS2_AVAILABLE:
        logger.error("ROS2 is not available, therefore all ROS2-Properties "
                     "will be just a normal Properties without connection to ROS2!")
        return Delete()

    # get config stuff
    node_name = ctx.conf(key=NODE_NAME_CONFIG_KEY)
    if not node_name:
        logger.error(f"{NODE_NAME_CONFIG_KEY} is not set. Shutting down ravestate_ros2")
        return Delete()
    spin_frequency = ctx.conf(key=SPIN_FREQUENCY_CONFIG_KEY)
    if spin_frequency is None:
        logger.error(f"{SPIN_FREQUENCY_CONFIG_KEY} is not set. Shutting down ravestate_ros2")
        return Delete()
    if spin_frequency == 0:
        spin_sleep_time = 0
    else:
        spin_sleep_time = 1 / spin_frequency

    # init ROS
    rclpy.init()
    node = rclpy.create_node(node_name)

    global global_prop_set
    current_props: Set = set()

    # ROS-Context Sync Loop
    while not ctx.shutting_down():
        # remove deleted props
        removed_props = current_props - global_prop_set
        for prop in removed_props:
            if isinstance(prop, Ros2SubProperty):
                node.destroy_subscription(prop.subscription)
            elif isinstance(prop, Ros2PubProperty):
                node.destroy_publisher(prop.publisher)
            current_props.remove(prop)

        # add new props
        new_props = global_prop_set - current_props
        for prop in new_props:
            # register subscribers in ROS
            if isinstance(prop, Ros2SubProperty):
                # register in context
                @receptor(ctx_wrap=ctx, write=prop.fullname())
                def ros_to_ctx_callback(ctx, msg, prop_name: str):
                    ctx[prop_name] = msg

                prop.ros_to_ctx_callback = ros_to_ctx_callback
                prop.subscription = node.create_subscription(prop.msg_type, prop.topic, prop.ros_subscription_callback)
            # register publishers in ROS
            if isinstance(prop, Ros2PubProperty):
                prop.publisher = node.create_publisher(prop.msg_type, prop.topic)

            # save in current props
            current_props.add(prop)

        # spin once
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(spin_sleep_time)

    node.destroy_node()
    rclpy.shutdown()


class Ros2SubProperty(PropertyBase):
    def __init__(self, name: str, topic: str, msg_type, default=None, always_signal_changed: bool = True):
        """
        Initialize Property
        :param name: Name of the property
        :param topic: ROS2-Topic that should be subscribed
        :param msg_type: ROS2-Message type of messages in the topic
        :param default: Default value of the property
        :param always_signal_changed: if signal:changed should be emitted if value is written again without changing
        """
        super().__init__(
            name=name,
            allow_read=True,
            allow_write=True,
            allow_push=False,
            allow_pop=False,
            default=default,
            always_signal_changed=always_signal_changed)
        self.topic = topic
        self.msg_type = msg_type
        self.subscription = None
        self.ros_to_ctx_callback = None
        global global_prop_set
        global_prop_set.add(self)

    def __del__(self):
        global global_prop_set
        global_prop_set.remove(self)

    def ros_subscription_callback(self, msg):
        """
        Writes the message from ROS to the property
        :param msg: ROS2-Message that should be written into the property
        """
        if self.ros_to_ctx_callback:
            logger.debug(f"{self.fullname()} received message {str(msg)} from topic {self.topic}")
            self.ros_to_ctx_callback(msg=msg, prop_name=self.fullname())


class Ros2PubProperty(PropertyBase):
    def __init__(self, name: str, topic: str, msg_type):
        """
        Initialize Property
        :param name: Name of the property
        :param topic: ROS2-Topic that messages should be pubished to
        :param msg_type: ROS2-Message type of messages in the topic
        """
        super().__init__(
            name=name,
            allow_read=True,
            allow_write=True,
            allow_push=False,
            allow_pop=False,
            default=None,
            always_signal_changed=False)
        self.topic = topic
        self.msg_type = msg_type
        self.publisher = None
        global global_prop_set
        global_prop_set.add(self)

    def __del__(self):
        global global_prop_set
        global_prop_set.remove(self)

    def write(self, value):
        """
        Publish value on ROS
        :param value: ROS2-Message that should be published
        """
        if super().write(value):
            if self.publisher:
                logger.debug(f"{self.fullname()} is publishing message {str(value)} on topic {self.topic}")
                self.publisher.publish(value)
            else:
                logger.error(f"Message {str(value)} on topic {self.topic} "
                             f"cannot be published because publisher was not registered in ROS")
