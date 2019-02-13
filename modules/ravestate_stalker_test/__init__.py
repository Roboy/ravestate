
from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate_ros2.ros2_properties import Ros2PubProperty
from ravestate.context import startup
from ravestate.state import state, s
from ravestate.wrappers import ContextWrapper

from ravestate_rawio import output as raw_in
from roboy_cognition_msgs.msg import RecognizedFaces

with Module(name="stalker_test"):

    face_pub = Ros2PubProperty(
        name="test_publisher",
        topic="/roboy/cognition/vision/visible_face_names",
        msg_type=RecognizedFaces)

    @state(cond=startup(min_age=3.), write=face_pub.id())
    def pub_some_name(ctx: ContextWrapper):

        ctx[face_pub.id()] = {
            "names": ["Steve"],
            "confidence": [100]
        }
