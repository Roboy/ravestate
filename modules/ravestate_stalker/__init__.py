
from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate_ros2.ros2_properties import Ros2SubProperty
from ravestate.context import startup
from ravestate.state import state, s
from ravestate.wrappers import ContextWrapper

from ravestate_rawio import output as raw_out

from reggol import get_logger
logger = get_logger(__name__)

ROBOY_COGNITION_AVAILABLE = False
try:
    from roboy_cognition_msgs.msg import RecognizedFaces
    ROBOY_COGNITION_AVAILABLE = True
except ImportError as e:
    logger.error(f"""
--------
An exception occurred during `from roboy_cognition_msgs.msg import RecognizedFaces`: {e}
Roboy will not comment on recognized faces!
--------
""")

if ROBOY_COGNITION_AVAILABLE:

    ROS2_FACE_TOPIC_CONFIG = "ros2-node"
    FACE_CONFIDENCE_THRESHOLD = "min-confidence"

    CONFIG = {
        ROS2_FACE_TOPIC_CONFIG: "/roboy/cognition/vision/visible_face_names",
        FACE_CONFIDENCE_THRESHOLD: 90
    }

    with Module(name="stalker", config=CONFIG) as mod:

        # Create a dummy parent, under which we can push the actual recognized faces topic,
        #  once a context with a configuration is available.
        subscriber_parent = PropertyBase(name="face_names_parent")

        @state(cond=startup(), write=subscriber_parent.id())
        def create_subscriber(ctx: ContextWrapper):
            
            face_names = Ros2SubProperty(
                "face_names",
                topic=ctx.conf(key=ROS2_FACE_TOPIC_CONFIG),
                msg_type=RecognizedFaces,
                always_signal_changed=False)

            ctx.push(subscriber_parent.id(), face_names)

            @state(read=face_names.id(), write=raw_out.id())
            def react_to_recognized_face(ctx: ContextWrapper):
                nonlocal face_names
                faces: RecognizedFaces = ctx[face_names.id()]

                best_name_and_confidence = "", 0
                for name_and_confidence in zip(faces.names, faces.confidence):

                    if name_and_confidence[1] > best_name_and_confidence[1]:
                        best_name_and_confidence = name_and_confidence

                if best_name_and_confidence[1] >= ctx.conf(key=FACE_CONFIDENCE_THRESHOLD):
                    ctx[raw_out.id()] = f"Hey, aren't you {best_name_and_confidence[0]}?!"

            mod.add(react_to_recognized_face)
            ctx.add_state(react_to_recognized_face)
