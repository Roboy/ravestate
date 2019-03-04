from datetime import datetime
from random import randint
from typing import List, Dict

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
        FACE_CONFIDENCE_THRESHOLD: 0.85
    }

    with Module(name="stalker", config=CONFIG) as mod:

        # Create a dummy parent, under which we can push the actual recognized faces topic,
        #  once a context with a configuration is available.
        subscriber_parent = PropertyBase(name="face_names_parent")

        @state(cond=startup(),  write=subscriber_parent.id())
        def create_subscriber(ctx: ContextWrapper):
            
            face_names = Ros2SubProperty(
                "face_names",
                topic=ctx.conf(key=ROS2_FACE_TOPIC_CONFIG),
                msg_type=RecognizedFaces,
                always_signal_changed=False)

            rec_faces = PropertyBase(name="rec_faces",
                 default_value={},
                 always_signal_changed=False,
                 allow_pop=True,
                 allow_push=True)

            ctx.push(subscriber_parent.id(), face_names)
            ctx.push(subscriber_parent.id(), rec_faces)

            @state(read=(face_names.id(), rec_faces.id()),  write=(rec_faces.id(), raw_out.id()))
            def react_to_recognized_face(ctx: ContextWrapper):
                nonlocal face_names
                faces: RecognizedFaces = ctx[face_names.id()]
                rec_faces_dict: Dict = ctx[rec_faces.id()]

                phrases: List = ["Hey, aren't you, {}?!",
                                 "How are you doing, {}?!",
                                 "How are you, {}?!",
                                 "Whats up, {}?!",
                                 "Nice to see you, {}?!",
                                 "Looking great today, {}!",
                                 "Hello, {}!",
                                 "Hi, {}!",
                                 "Greetings, {}!",
                                 "Howdy, {}!",
                                 "Hey, {}!",
                                 "Greetings to {} over here!",
                                 "Hi there, {}!",
                                 "Gruse gott, {}!"]

                best_name_and_confidence = "", 0
                for name_and_confidence in zip(faces.names, faces.confidence):

                    logger.info(str(name_and_confidence[0]))
                    if name_and_confidence[1] > best_name_and_confidence[1]:
                        best_name_and_confidence = name_and_confidence

                if best_name_and_confidence[1] >= ctx.conf(key=FACE_CONFIDENCE_THRESHOLD):
                    if best_name_and_confidence[0] not in rec_faces_dict.keys() or \
                            datetime.timestamp(datetime.now()) - rec_faces_dict.get(best_name_and_confidence[0]) > 300:
                        rec_faces_dict.update({best_name_and_confidence[0]: datetime.timestamp(datetime.now())})
                        ctx[raw_out.id()] = phrases[randint(0, len(phrases) - 1)].format(best_name_and_confidence[0])

                    ctx[rec_faces.id()] = rec_faces_dict

            mod.add(react_to_recognized_face)
            ctx.add_state(react_to_recognized_face)
