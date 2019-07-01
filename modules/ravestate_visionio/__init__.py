import ravestate as rs
import ravestate_rawio as rawio
import ravestate_interloc as interloc
import ravestate_ontology

from scientio.session import Session
from scientio.ontology.ontology import Ontology
from scientio.ontology.node import Node
from ravestate_visionio.face_vec_generator import detect_face
import redis

from reggol import get_logger
logger = get_logger(__name__)

PYROBOY_AVAILABLE = False
try:
    from pyroboy.face_recognition import FaceRec
    PYROBOY_AVAILABLE = True
except ImportError as e:
    import face_recognition as fr

REDIS_HOST_CONF = "redis_host"
REDIS_PORT_CONF = "redis_port"
REDIS_PASS_CONF = "redis_pass"
CONFIG = {
    REDIS_HOST_CONF: "localhost",
    REDIS_PORT_CONF: 6379,
    REDIS_PASS_CONF: None
}

with rs.Module(name="visionio") as mod:
    prop_person_in_vision = rs.Property(
        name="person_in_vision",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    prop_face_vector = rs.Property(
        name="face_vector",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    @rs.state(
        cond=rawio.prop_in.changed(),
        write=prop_person_in_vision
    )
    def live_demo(ctx: rs.ContextWrapper):
        rawio_input = ctx[rawio.prop_in]
        if rawio_input=='live demo':
            ctx[prop_person_in_vision] = detect_face()

    @rs.state(
        cond=prop_person_in_vision.changed(),
        write=(),
        read=(prop_face_vector))
    def recognize(ctx: rs.ContextWrapper):
        face_vector = ctx[prop_face_vector]

        #TODO LEARN ABOUT REDIS STRUCTURE
        def get_person_face_vector_pairs() -> list:
            try:
                redis_conn = redis.Redis(
                    host=ctx.conf(key=REDIS_HOST_CONF),
                    port=ctx.conf(key=REDIS_PORT_CONF),
                    password=ctx.conf(key=REDIS_PASS_CONF))
                # name_keys = redis_conn.keys('name')
                # IMPLEMENT HERE
                return list()
            except redis.exceptions.ConnectionError as e:
                err_msg = "Looks like the redis connection is unavailable :-("
                logger.error(err_msg)
                ctx[rawio.prop_out] = err_msg

        people = get_person_face_vector_pairs()
        index, _ = FaceRec.match_face(face_vector, [person['name'] for person in people])

        #TODO: Change the condition later
        if index > -1:
            person = people[index]
            name = person[0]
            person_node = next((person for person in people if person.get_name() == name), None)

            onto: Ontology = ravestate_ontology.get_ontology()
            sess: Session = ravestate_ontology.get_session()
            people_node_type = Node(metatype=onto.get_type("Person"))
            people = sess.retrieve(request=people_node_type)
        else:
            person_node = Node()
            person_node.set_properties({'face_vector': face_vector})

        # TODO: Ask Joseph about this: push it to interloc.prop_all???
        if ctx.push(parent_property_or_path=interloc.prop_all,
                    child=rs.Property(name=name, default_value=person_node)):
            logger.debug(f"Pushed {person_node} to interloc:all")





