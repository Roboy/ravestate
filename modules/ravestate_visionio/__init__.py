import ravestate as rs
import ravestate_rawio as rawio
import ravestate_interloc as interloc
import ravestate_ontology as mem
import ravestate_persqa as persqa
import ravestate_ros1 as ros

from scientio.session import Session
from scientio.ontology.ontology import Ontology
from scientio.ontology.node import Node
import redis

from reggol import get_logger
logger = get_logger(__name__)

PYROBOY_AVAILABLE = False
try:
    from pyroboy.face_recognition import FaceRec
    PYROBOY_AVAILABLE = True
except ImportError as e:
    import face_recognition as fr

ROBOY_COGNITION_AVAILABLE = False
try:
    from roboy_cognition_msgs.msg import Faces
    ROBOY_COGNITION_AVAILABLE = True
except ImportError as e:
    logger.error(f"""
--------
An exception occurred during `from roboy_cognition_msgs.msg import Faces`: {e}
Roboy will not comment on recognized faces!
--------
""")

if ROBOY_COGNITION_AVAILABLE:

    REDIS_HOST_CONF = "redis_host"
    REDIS_PORT_CONF = "redis_port"
    REDIS_PASS_CONF = "redis_pass"
    ROS1_FACE_TOPIC_CONFIG = "ros1-node"
    FACE_CONFIDENCE_THRESHOLD = "min-confidence"

    CONFIG = {
        REDIS_HOST_CONF: "localhost",
        REDIS_PORT_CONF: 6379,
        REDIS_PASS_CONF: None,
        ROS1_FACE_TOPIC_CONFIG: "/roboy/cognition/vision/visible_face_names",
        FACE_CONFIDENCE_THRESHOLD: 0.85
    }

    with rs.Module(name="visionio", config=CONFIG) as mod:

        prop_subscribe_faces = ros.Ros1SubProperty(
            "face_names",
            topic="/roboy/cognition/vision/visible_face_names",
            msg_type=Faces,
            always_signal_changed=False)

        @rs.state(
            cond=prop_subscribe_faces.changed().detached(),
            write=(rawio.prop_out, interloc.prop_all),
            read=(prop_subscribe_faces, interloc.prop_all),
        )
        def recognize_faces(ctx: rs.ContextWrapper):
            if not ctx.properties[interloc.prop_all].prop.children:
                faces: Faces = ctx[prop_subscribe_faces]

                name = faces.names[0]
                confidence = faces.confidence[0]
                face_encodings = faces.face_encodings

                onto: Ontology = mem.get_ontology()
                sess: Session = mem.get_session()

                person_node = Node(metatype=onto.get_type("Person"))

                if confidence and confidence > ctx.conf(key=FACE_CONFIDENCE_THRESHOLD):
                    person_node.set_properties({'name': name})
                    person_node = sess.retrieve(request=person_node)[0]
                else:
                    person_node.set_properties({'face_vector': face_encodings})
                if not person_node:
                    err_msg = "Person with name %s is not found in memory." % name
                    logger.error(err_msg)
                    ctx[rawio.prop_out] = err_msg
                    return

                pushed = ctx.push(parent_property_or_path=interloc.prop_all,
                            child=rs.Property(name=interloc.ANON_INTERLOC_ID, default_value=person_node))
                if pushed:
                    logger.debug(f"Pushed {person_node} to interloc:all")
                else:
                    err_msg = "Looks like connection to pyroboy module is unavailable :-("
                    logger.error(err_msg)
                    ctx[rawio.prop_out] = err_msg


        @rs.state(
            cond=interloc.prop_persisted.changed(),
            read=interloc.prop_persisted
        )
        def save_person_in_vision(ctx: rs.ContextWrapper):
            node: Node = ctx[interloc.prop_persisted]
            encodings = node.get_properties('face_vector')

            sess: Session = mem.get_session()
            node.set_properties({'face_vector': None})
            sess.update(node)

            try:
                redis_conn = redis.Redis(
                    host=ctx.conf(key=REDIS_HOST_CONF),
                    port=ctx.conf(key=REDIS_PORT_CONF),
                    password=ctx.conf(key=REDIS_PASS_CONF))
                redis_conn.set(node.get_id(), str(encodings))
            except redis.exceptions.ConnectionError as e:
                err_msg = "Looks like the redis connection is unavailable :-("
                logger.error(err_msg)