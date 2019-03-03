from ravestate.module import Module
from ravestate.state import s, state, Emit
from ravestate.property import PropertyBase
from ravestate.wrappers import ContextWrapper
import ravestate_ontology
import pickle
import redis

import ravestate_rawio
import ravestate_nlp
import ravestate_idle

from scientio.ontology.node import Node
from scientio.session import Session
from scientio.ontology.ontology import Ontology

from ravestate_sendpics.face_recognition import recognize_face_from_image_file

from reggol import get_logger
logger = get_logger(__name__)

REDIS_HOST_CONF = "redis_host"
REDIS_PORT_CONF = "redis_port"
REDIS_PASS_CONF = "redis_pass"
CONFIG = {
    REDIS_HOST_CONF: "localhost",
    REDIS_PORT_CONF: 6379,
    REDIS_PASS_CONF: None
}


with Module(name="sendpics", config=CONFIG):

    face_vec = PropertyBase(name="face_vec", always_signal_changed=True)

    @state(cond=s("idle:bored"), write="rawio:out", weight=1.2, cooldown=30.)
    def prompt_send(ctx):
        ctx["rawio:out"] = "Why don't you send me a picture? I'm really good at recognizing faces!"

    @state(
        read="rawio:pic_in",
        write=("rawio:out", "sendpics:face_vec"),
        emit_detached=True)
    def prompt_name(ctx):
        # Get face ancoding
        face = recognize_face_from_image_file(ctx["rawio:pic_in"])
        # Prompt name
        if face is not None:
            ctx["rawio:out"] = "Nice picture! Who is that person?"
            ctx["sendpics:face_vec"] = pickle.dumps(face)
        else:
            ctx["rawio:out"] = f"Huh, looks interesting. But maybe send me a cute face!"

    @state(
        cond=(
            s("sendpics:repeat_name", max_age=30.) | s("sendpics:face_vec:changed", max_age=30.)
        ) & s("nlp:tokens:changed"),
        signal_name="repeat_name",
        read=("nlp:triples", "nlp:tokens", "sendpics:face_vec"),
        write="rawio:out",
        emit_detached=True)
    def store_face_and_name(ctx: ContextWrapper):
        tokens = ctx["nlp:tokens"]
        triples = ctx["nlp:triples"]
        if len(tokens) == 1:
            name = tokens[0]
        elif triples[0].get_object().text and triples[0].match_either_lemma(pred={"be"}):
            name = triples[0].get_object().text
        else:
            ctx["rawio:out"] = "Sorry, what was the name?"
            return Emit()
        ctx["rawio:out"] = f"Got it, I'm sure I'll remember {name} next time I see that face!"

        # Create memory entry
        sess: Session = ravestate_ontology.get_session()
        onto: Ontology = ravestate_ontology.get_ontology()
        query = Node(metatype=onto.get_type("Person"))
        query.set_properties({"name": name})
        node_list = sess.retrieve(query)
        if not node_list:
            node = sess.create(query)
            logger.info(f"Created new Node in scientio session: {node}")
        elif len(node_list) == 1:
            node = node_list[0]
        else:
            logger.error(f'Failed to create or retrieve Scientio Node for {name}!')
            return
        logger.info(f"Node ID for {name} in picture is {node.get_id()}!")

        # Store face vector with node id in redis
        try:
            redis_conn = redis.Redis(
                host=ctx.conf(key=REDIS_HOST_CONF),
                port=ctx.conf(key=REDIS_PORT_CONF),
                password=ctx.conf(key=REDIS_PASS_CONF))
            redis_conn.set(node.get_id(), ctx["sendpics:face_vec"])
        except redis.exceptions.ConnectionError as e:
            err_msg = "Looks like the redis connection is unavailable :-("
            logger.error(err_msg)
            ctx['rawio:out'] = err_msg
