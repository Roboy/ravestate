from testfixtures import LogCapture
from ravestate.testfixtures import *
from pytest_mock import mocker

from ravestate.context import Context
from ravestate.state import Delete

FILE_NAME = 'ravestate_ros2.ros2_properties'
PREFIX = f"[{FILE_NAME}] [\x1b[1;36m{FILE_NAME}\x1b[0m]"


def test_sync_ros_properties_no_ros2(mocker, context_wrapper_fixture: ContextWrapper):
    with LogCapture() as capture:
        mocker.patch.dict('sys.modules', {'rclpy': None})
        from ravestate_ros2 import sync_ros_properties
        result = sync_ros_properties(context_wrapper_fixture)
        expected = "ROS2 is not available, therefore all ROS2-Properties " \
                   "will be just normal properties without connection to ROS2!"
        capture.check_present((f"{FILE_NAME}", '\x1b[1;31mERROR\x1b[0m', f"{PREFIX} {expected}"))
        assert isinstance(result, Delete)


def test_sync_ros_properties(mocker, context_wrapper_fixture: ContextWrapper):
    with LogCapture() as capture:
        rclpy_mock = mocker.Mock()
        mocker.patch.dict('sys.modules', {'rclpy': rclpy_mock})
        from ravestate_ros2 import sync_ros_properties
        result = sync_ros_properties(context_wrapper_fixture)
        expected = "ros2-node-name is not set. Shutting down ravestate_ros2"
        capture.check_present((f"{FILE_NAME}", '\x1b[1;31mERROR\x1b[0m', f"{PREFIX} {PREFIX} {expected}"))
