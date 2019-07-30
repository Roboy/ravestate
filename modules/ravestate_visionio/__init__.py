import ravestate as rs
import ravestate_rawio as rawio
import ravestate_interloc as interloc
import ravestate_ontology as mem
import ravestate_persqa as persqa
import ravestate_ros1 as ros
from ravestate_visionio.faceoraclefilter import FaceOracleFilter, Person
from scientio.session import Session
from scientio.ontology.ontology import Ontology
from scientio.ontology.node import Node
import redis

from reggol import get_logger
logger = get_logger(__name__)

ROBOY_COGNITION_AVAILABLE = False
try:
    from roboy_cognition_msgs.msg import Faces, FacialFeatures
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
            always_signal_changed=False
        )

        prop_face_filter = rs.Property(
            name="face_frequencies",
            default_value=FaceOracleFilter(),
            always_signal_changed=True,
            allow_write=True
        )

        @rs.state(
            cond=prop_subscribe_faces.changed().detached(),
            write=(rawio.prop_out, interloc.prop_all, prop_face_filter),
            read=(prop_subscribe_faces, interloc.prop_all, prop_face_filter),
        )
        def recognize_faces(ctx: rs.ContextWrapper):
            face_filter = ctx[prop_face_filter]

            faces: Faces = ctx[prop_subscribe_faces]

            # Push faces to face filter
            best_guess_changed = face_filter.push_message(faces)

            if best_guess_changed:
                current_best_guess: Person = face_filter.current_best_guess

                onto: Ontology = mem.get_ontology()
                sess: Session = mem.get_session()

                person_node = Node(metatype=onto.get_type("Person"))

                best_guess_id = current_best_guess.id
                face_vector = current_best_guess.face_vector

                if current_best_guess.is_known:

                    person_node.set_id(best_guess_id)
                    person_node_query = sess.retrieve(request=person_node)
                    if person_node_query:
                        person_node = person_node_query[0]
                    else:
                        err_msg = "Person with id %s is not found in memory." % best_guess_id
                        logger.error(err_msg)
                        ctx[rawio.prop_out] = err_msg
                        return
                else:
                    person_node.set_properties({'face_vector': face_vector})

                push = False

                # Check if there is any interlocutor. If necessary and pop the current node and push person node
                # instead.
                if any(ctx.enum(interloc.prop_all)):
                    interloc_node: Node = ctx[f'interloc:all:{interloc.ANON_INTERLOC_ID}']

                    # If interloc and the person nodes are not same pop and push person node.
                    if not (interloc_node.get_id() == person_node.get_id()) or interloc_node.get_id() < 0:

                        # Remove the current interloc
                        logger.info('Popping current interlocutor')
                        popped_node = ctx.pop(f'interloc:all:{interloc.ANON_INTERLOC_ID}')
                        assert popped_node == True
                        push = True

                    else:
                        # Update the face vector of the person already familiar with
                        try:
                            redis_conn = redis.Redis(
                                host=ctx.conf(key=REDIS_HOST_CONF),
                                port=ctx.conf(key=REDIS_PORT_CONF),
                                password=ctx.conf(key=REDIS_PASS_CONF))
                            redis_conn.set(interloc_node.get_id(), str(current_best_guess.face_vector))
                            logger.info('Updating familiar person face')
                        except redis.exceptions.ConnectionError as e:
                            err_msg = "Looks like the redis connection is unavailable :-("
                            logger.error(err_msg)
                else:
                    push = True

                if push:
                    # Push the new interlocutor
                    if ctx.push(parent_property_or_path=interloc.prop_all,
                            child=rs.Property(name=interloc.ANON_INTERLOC_ID, default_value=person_node)):
                        logger.info(f"Pushed {person_node} to interloc:all")
                    else:
                        err_msg = "Interlocutor push is unsuccessful!"
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

            save_face(ctx, node.get_id(), encodings)

        def save_face(ctx: rs.ContextWrapper, id, face_vector):
            try:
                redis_conn = redis.Redis(
                    host=ctx.conf(key=REDIS_HOST_CONF),
                    port=ctx.conf(key=REDIS_PORT_CONF),
                    password=ctx.conf(key=REDIS_PASS_CONF))
                redis_conn.set(id, str(face_vector))
            except redis.exceptions.ConnectionError as e:
                err_msg = "Looks like the redis connection is unavailable :-("
                logger.error(err_msg)