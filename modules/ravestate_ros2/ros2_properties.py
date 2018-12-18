import rclpy

from ravestate.property import PropertyBase
import threading

from reggol import get_logger
logger = get_logger(__name__)

# TODO destroy nodes

class Ros2SubProperty(PropertyBase):
    # TODO use kwargs for super params
    def __init__(self, name: str, topic: str, msg_type):  # TODO annotate rosmsg
        super().__init__(name=name)
        self.topic = topic
        self.msg_type = msg_type
        t = threading.Thread(target=self.subscribe)
        t.start()

    def subscriber_callback(self, msg):
        """
        Writes the message data to the property
        """
        logger.debug(f"Reveived msg {msg.data} from topic {self.topic}")
        self.write(msg.data)

    def subscribe(self):
        subscriber_node = rclpy.create_node('sub_prop_' + self.topic)
        self.subscription = subscriber_node.create_subscription(self.msg_type, self.topic, self.subscriber_callback)
        rclpy.spin(subscriber_node)


class Ros2PubProperty(PropertyBase):
    def __init__(self, name: str, topic: str, msg_type):  # TODO annotate rosmsg
        super().__init__(name=name)
        self.topic = topic
        self.msg_type = msg_type
        self.publish()

    def publish(self):
        self.publisher_node = rclpy.create_node('sub_prop_' + self.topic)
        self.publisher = self.publisher_node.create_publisher(self.msg_type, self.topic)
        #rclpy.spin(publisher_node)

    def write(self, value):
        if super().write(value):
            logger.debug(f"About to publish {value.data} on topic {self.topic}")
            self.publisher.publish(value)
            rclpy.spin_once(self.publisher_node)

