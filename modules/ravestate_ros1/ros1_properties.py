from typing import Dict

import ravestate as rs

from reggol import get_logger
logger = get_logger(__name__)


ROS1_AVAILABLE = True
try:
    import rospy
except ImportError:
    logger.error("Could not import rospy. Please make sure to have ROS1 installed.")
    ROS1_AVAILABLE = False

NODE_NAME_CONFIG_KEY = "ros1-node-name"
SPIN_FREQUENCY_CONFIG_KEY = "ros1-spin-frequency"

global_prop_set = set()
global_node = None


@rs.state(cond=rs.sig_startup)
def sync_ros_properties(ctx: rs.ContextWrapper):
    """
    State that creates a ROS1-Node, registers all Ros1SubProperties and Ros1PubProperties in ROS1 and keeps them synced
    """
    global global_prop_set, global_node

    # check for ROS1 availability
    if not ROS1_AVAILABLE:
        logger.error("ROS1 is not available, therefore all ROS1-Properties "
                     "will be just normal properties without connection to ROS1!")
        return rs.Delete()

    # get config stuff
    node_name = ctx.conf(key=NODE_NAME_CONFIG_KEY)
    if not node_name:
        logger.error(f"{NODE_NAME_CONFIG_KEY} is not set. Shutting down ravestate_ros1")
        return rs.Delete()
    spin_frequency = ctx.conf(key=SPIN_FREQUENCY_CONFIG_KEY)
    if spin_frequency is None or spin_frequency < 0:
        logger.error(f"{SPIN_FREQUENCY_CONFIG_KEY} is not set or less than 0. Shutting down ravestate_ros1")
        return rs.Delete()
    if spin_frequency == 0:
        spin_sleep_time = 0
    else:
        spin_sleep_time = 1 / spin_frequency

    # Use same node_name if ROS1 was already initialized (i.e. by importing pyroboy)
    if rospy.get_name():
        node_name = rospy.get_name()[1:]  # cut off leading /
    # init ROS1
    rospy.init_node(node_name, disable_signals = True)
    global global_prop_set
    # current_props: hash -> Subscriber/Publisher/ServiceProxy
    current_props: Dict = dict()

    # ROS1-Context Sync Loop
    while not ctx.shutting_down() and not rospy.core.is_shutdown():
        # remove deleted props
        removed_props = current_props.keys() - global_prop_set
        for prop_hash in removed_props:
            item = current_props[prop_hash]
            item.unregister()
            current_props.pop(prop_hash)

        # add new props
        new_props = global_prop_set - current_props.keys()
        for prop in new_props:
            # register subscribers in ROS1
            if isinstance(prop, Ros1SubProperty):
                # register in context
                @rs.receptor(ctx_wrap=ctx, write=prop.id())
                def ros_to_ctx_callback(ctx, msg, prop_name: str):
                    ctx[prop_name] = msg

                prop.ros_to_ctx_callback = ros_to_ctx_callback
                prop.subscriber = rospy.Subscriber(prop.topic, prop.msg_type, prop.ros_subscription_callback)
                current_props[prop.__hash__()] = prop.subscriber
            # register publishers in ROS1
            if isinstance(prop, Ros1PubProperty):
                prop.publisher = rospy.Publisher(prop.topic, prop.msg_type, queue_size=prop.queue_size)
                current_props[prop.__hash__()] = prop.publisher
            # register clients in ROS1
            if isinstance(prop, Ros1CallProperty):
                prop.client = rospy.ServiceProxy(prop.service_name, prop.service_type)
                current_props[prop.__hash__()] = prop.client

            # replace prop with hash in global_props
            global_prop_set.remove(prop)
            global_prop_set.add(prop.__hash__())

        rospy.rostime.wallsleep(spin_sleep_time)

    rospy.signal_shutdown("ravestate_ros1 is shutting down")


class Ros1SubProperty(rs.Property):
    def __init__(self, name: str, topic: str, msg_type, default_value=None, always_signal_changed: bool = True):
        """
        Initialize Property

        * `name`: Name of the property

        * `topic`: ROS1-Topic that should be subscribed

        * `msg_type`: ROS1-Message type of messages in the topic

        * `default_value`: Default value of the property

        * `always_signal_changed`: if signal:changed should be emitted if value is written again without changing
        """
        super().__init__(
            name=name,
            allow_read=True,
            allow_write=True,
            allow_push=False,
            allow_pop=False,
            default_value=default_value,
            always_signal_changed=always_signal_changed)
        self.topic = topic
        self.msg_type = msg_type
        self.subscriber = None
        self.ros_to_ctx_callback = None
        global global_prop_set
        global_prop_set.add(self)

    def __del__(self):
        global global_prop_set
        global_prop_set.remove(self.__hash__())

    def clone(self):
        result = Ros1SubProperty(
            name=self.name,
            topic=self.topic,
            msg_type=self.msg_type,
            default_value=self.value,
            always_signal_changed=self.always_signal_changed)
        result.set_parent_path(self.parent_path)
        return result

    def ros_subscription_callback(self, msg):
        """
        Writes the message from ROS1 to the property

        * `msg`: ROS1-Message that should be written into the property
        """
        if self.ros_to_ctx_callback:
            logger.debug(f"{self.id()} received message {str(msg)} from topic {self.topic}")
            self.ros_to_ctx_callback(msg=msg, prop_name=self.id())


class Ros1PubProperty(rs.Property):
    def __init__(self, name: str, topic: str, msg_type, queue_size: int = 10):
        """
        Initialize Property

        * `name`: Name of the property

        * `topic`: ROS1-Topic that messages should be pubished to

        * `msg_type`: ROS1-Message type of messages in the topic

        * `queue_size`: Limits the amount of queued messages if any ROS1-Subscriber is not receiving them fast enough
        """
        super().__init__(
            name=name,
            allow_read=True,
            allow_write=True,
            allow_push=False,
            allow_pop=False,
            default_value=None,
            always_signal_changed=True)
        self.topic = topic
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.publisher = None
        global global_prop_set
        global_prop_set.add(self)

    def __del__(self):
        global global_prop_set
        global_prop_set.remove(self.__hash__())

    def clone(self):
        result = Ros1PubProperty(
            name=self.name,
            topic=self.topic,
            msg_type=self.msg_type)
        result.set_parent_path(self.parent_path)
        return result

    def write(self, value):
        """
        Publish value on ROS1

        * `value`: ROS1-Message that should be published
        """
        if super().write(value):
            if self.publisher:
                logger.debug(f"{self.id()} is publishing message {str(value)} on topic {self.topic}")
                self.publisher.publish(value)
            elif ROS1_AVAILABLE:
                logger.error(f"Message {str(value)} on topic {self.topic} "
                             f"cannot be published because publisher was not registered in ROS1")


class Ros1CallProperty(rs.Property):
    def __init__(self, name: str, service_name: str, service_type, call_timeout: float = 10.0):
        """
        Initialize Property

        * `name`: Name of the property

        * `service_name`: ROS1-Service that should be called

        * `service_type`: Type of the service used

        * `call_timeout`: Timeout when waiting for availability of a service in seconds
        """
        super().__init__(
            name=name,
            allow_read=True,
            allow_write=True,
            allow_push=False,
            allow_pop=False,
            default_value=None,
            always_signal_changed=False)
        self.service_name = service_name
        self.service_type = service_type
        self.call_timeout = call_timeout
        self.client = None
        global global_prop_set
        global_prop_set.add(self)

    def __del__(self):
        global global_prop_set
        global_prop_set.remove(self.__hash__())

    def clone(self):
        result = Ros1CallProperty(
            name=self.name,
            service_name=self.service_name,
            service_type=self.service_type,
            call_timeout=self.call_timeout)
        result.set_parent_path(self.parent_path)
        return result

    def write(self, value):
        """
        Call Service and receive result directly in the property.
        Blocks during service-call.
        If service is not available, writes None into the property value and returns.

        * `value`: Either Request-Object for the service or dict containing the attributes of the Request
        """
        if super().write(value):
            if self.client:
                try:
                    rospy.wait_for_service(self.service_name, timeout=self.call_timeout)
                except rospy.ROSException:
                    logger.error(f'service {self.service_name} not available')
                    super().write(None)
                    return

                logger.debug(f"{self.id()} is sending request {str(value)} to service {self.service_name}")
                if isinstance(value, dict):
                    # value is dict {"param1": value1, "param2": value2}
                    result = self.client.call(**value)
                elif isinstance(value, tuple) or isinstance(value, list):
                    # value is ordered sequence (value1, value2)
                    result = self.client.call(*value)
                else:
                    # value should be of matching Request-Type
                    result = self.client.call(value)

                super().write(result)

            elif ROS1_AVAILABLE:
                logger.error(f"Request {str(value)} to service {self.service_name} "
                             f"cannot be sent because client was not registered in ROS1")

