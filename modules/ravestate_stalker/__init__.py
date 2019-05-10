from datetime import datetime
from random import randint
from typing import List, Dict

import ravestate as rs
import ravestate_rawio as rawio
import ravestate_ros2 as ros2

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

    with rs.Module(name="stalker", config=CONFIG) as mod:

        # Create a dummy parent, under which we can push the actual recognized faces topic,
        #  once a context with a configuration is available.
        subscriber_parent = rs.Property(name="face_names_parent")

        @rs.state(cond=rs.sig_startup, write=subscriber_parent)
        def create_subscriber(ctx: rs.ContextWrapper):

            face_names = ros2.Ros2SubProperty(
                "face_names",
                topic=ctx.conf(key=ROS2_FACE_TOPIC_CONFIG),
                msg_type=RecognizedFaces,
                always_signal_changed=False)

            rec_faces = rs.Property(
                name="rec_faces",
                default_value={},
                always_signal_changed=False,
                allow_pop=True,
                allow_push=True)

            ctx.push(subscriber_parent, face_names)
            ctx.push(subscriber_parent, rec_faces)

            @rs.state(read=(face_names, rec_faces), write=(rec_faces, rawio.prop_out), signal="daddy")
            def react_to_recognized_face(ctx: rs.ContextWrapper):
                nonlocal face_names
                faces: RecognizedFaces = ctx[face_names]
                rec_faces_dict: Dict = ctx[rec_faces]

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
                        ctx[rawio.prop_out] = phrases[randint(0, len(phrases) - 1)].format(best_name_and_confidence[0])

                    ctx[rec_faces] = rec_faces_dict

                if best_name_and_confidence[0].lower() == "daddy":
                    return rs.Emit()

            mod.add(react_to_recognized_face)
            ctx.add_state(react_to_recognized_face)
