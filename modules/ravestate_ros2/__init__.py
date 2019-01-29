from ravestate.module import Module
from ravestate_ros2.ros2_properties import sync_ros_properties

CONFIG = {
    # name of the ROS2-Node that is created by ravestate_ros2
    ros2_properties.NODE_NAME_CONFIG_KEY: "ravestate_ros2",
    # frequency for spinning of ROS2-Node in spins per second (0: as fast as possible)
    ros2_properties.SPIN_FREQUENCY_CONFIG_KEY: 10
}

with Module(name="ros2", config=CONFIG) as mod:
    mod.add(sync_ros_properties)
