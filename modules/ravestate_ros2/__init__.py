from ravestate import registry
from ravestate_ros2.ros2_properties import register_ros_subscribers

registry.register(
    name="ros2",
    states=(register_ros_subscribers,))
