import ravestate as rs
import ravestate_interloc as interloc
import ravestate_ontology as mem
import ravestate_ros1 as ros

from ravestate_visionio.faceoraclefilter import FaceOracleFilter, Person
from scientio.session import Session
from scientio.ontology.ontology import Ontology
from scientio.ontology.node import Node
import redis
import pickle

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
    PERSON_DISAPPEARED_THRESHOLD = "person-disappeared-threshold"

    CONFIG = {
        REDIS_HOST_CONF: "localhost",
        REDIS_PORT_CONF: 6379,
        REDIS_PASS_CONF: "",
        ROS1_FACE_TOPIC_CONFIG: "/roboy/cognition/vision/visible_face_names",
        FACE_CONFIDENCE_THRESHOLD: 0.85,
        PERSON_DISAPPEARED_THRESHOLD: 5
    }

    with rs.Module(name="visionio", config=CONFIG, depends=(interloc.mod, mem.mod, ros.mod)) as mod:

        prop_subscribe_faces = ros.Ros1SubProperty(
            "face_names",
            topic="/roboy/cognition/vision/visible_face_names",
            msg_type=Faces,
            always_signal_changed=True,
            wipe_on_changed=False,
            boring=True
        )

        prop_face_filter = rs.Property(
            name="face_frequencies",
            default_value=None,
            always_signal_changed=True,
            allow_write=True
        )

        @rs.state(
            cond=rs.sig_startup,  # | prop_subscribe_faces
            # TODO: Reset during runtime is disabled for now. Reason:
            #  Camera image stream is too unreliable for timeout-mechanic to be
            #  reasonable, and there is a conflict between `bye` and vision-based
            #  conversation stop.
            # .changed()
            # .detached()  # detached, because recognize_faces is also listening.
            # .min_age(rs.ConfigurableAge(key=PERSON_DISAPPEARED_THRESHOLD))
            # .max_age(-1),
            read=(rs.prop_activity, interloc.prop_all, interloc.prop_persisted),
            write=(prop_face_filter, interloc.prop_all),
            boring=True
        )
        def reset(ctx: rs.ContextWrapper):
            """
            If there is no call for given seconds, removes the interlocutor node and initializes face oracle filter
            from scratch.
            """
            # Remove interloc if present
            if any(ctx.enum(interloc.prop_all)):
                popped_node = ctx.pop(f'interloc:all:{interloc.ANON_INTERLOC_ID}')
                if popped_node:
                    logger.info("Visual contact is broken, removed the interlocutor node")
            # Install a new FaceOracleFilter
            ctx[prop_face_filter] = FaceOracleFilter()

        @rs.state(
            cond=prop_subscribe_faces.changed(),
            write=interloc.prop_all,
            read=(prop_subscribe_faces, interloc.prop_all, prop_face_filter)
        )
        def recognize_faces(ctx: rs.ContextWrapper):
            """
            Activates with each incoming face data served by face oracle. Responsible for synchronizing the node of
            person in vision with the anonymous interlocutor node. Uses face oracle filter to organize the incoming data
            and find out the right person.
            """

            face_filter: FaceOracleFilter = ctx[prop_face_filter]
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

                    person_node_query = sess.retrieve(node_id=best_guess_id)
                    if person_node_query:
                        person_node = person_node_query[0]
                    else:
                        err_msg = "Person with id %s is not found in memory." % best_guess_id
                        logger.error(err_msg)
                        return
                else:
                    person_node.set_properties({
                        'face_vector': face_vector,
                        'name': interloc.ANON_INTERLOC_ID
                    })

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
                        save_face(ctx, interloc_node.get_id(), current_best_guess.face_vector)
                else:
                    push = True
                if push:
                    # Push the new interlocutor
                    ctx.push(
                        parent_property_or_path=interloc.prop_all,
                        child=rs.Property(
                            name=interloc.ANON_INTERLOC_ID,
                            default_value=person_node))
                    logger.info(f"Pushed node with id {person_node.id} to interloc:all")


        @rs.state(
            cond=interloc.prop_persisted.changed(),
            read=(interloc.prop_persisted, prop_face_filter)
        )
        def save_person_in_vision(ctx: rs.ContextWrapper):
            """
            Activates if the anonymous interlocutor's name is given and saved to database. Responsible for saving the
            current interlocutor's face vector to redis and setting the current anonymous person in face filter to known
            """

            node: Node = ctx[interloc.prop_persisted]
            encodings = node.get_properties('face_vector')

            face_filter = ctx[prop_face_filter]

            sess: Session = mem.get_session()
            node.set_properties({'face_vector': None})
            sess.update(node)

            face_filter.convert_current_anonymous_to_known(node.get_id())
            save_face(ctx, node.get_id(), encodings)

        def save_face(ctx: rs.ContextWrapper, id, face_vector):
            try:
                redis_conn = redis.Redis(
                    host=ctx.conf(key=REDIS_HOST_CONF),
                    port=ctx.conf(key=REDIS_PORT_CONF),
                    password=ctx.conf(key=REDIS_PASS_CONF))
                redis_conn.set(id, pickle.dumps(face_vector))
            except redis.exceptions.ConnectionError as e:
                err_msg = "Looks like the redis connection is unavailable :-("
                logger.error(err_msg)
