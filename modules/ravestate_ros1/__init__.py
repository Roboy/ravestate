import ravestate as rs

from ravestate_ros1.ros1_properties import sync_ros_properties, Ros1SubProperty, Ros1PubProperty, Ros1CallProperty

CONFIG = {
    # name of the ROS-Node that is created by ravestate_ros1
    ros1_properties.NODE_NAME_CONFIG_KEY: "ravestate_ros1",
    # frequency for spinning of ROS-Node in spins per second (0: as fast as possible)
    ros1_properties.SPIN_FREQUENCY_CONFIG_KEY: 10
}

with rs.Module(name="ros1", config=CONFIG) as mod:

    mod.add(sync_ros_properties)
