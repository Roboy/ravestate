import ravestate as rs
import ravestate_ontology
import pickle
import redis

import ravestate_rawio as rawio
import ravestate_nlp as nlp
import ravestate_idle as idle

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


with rs.Module(name="sendpics", config=CONFIG):

    prop_face_vec = rs.Property(name="face_vec", always_signal_changed=True)

    sig_repeat_name = rs.Signal("repeat_name")

    @rs.state(cond=idle.sig_bored, write=rawio.prop_out, weight=1.2, cooldown=30.)
    def prompt_send(ctx):
        ctx[rawio.prop_out] = "Why don't you send me a picture? I'm really good at recognizing faces!"

    @rs.state(
        read=rawio.prop_pic_in,
        write=(rawio.prop_out, prop_face_vec),
        emit_detached=True)
    def prompt_name(ctx):
        # Get face ancoding
        face = recognize_face_from_image_file(ctx[rawio.prop_pic_in])
        # Prompt name
        if face is not None:
            ctx[rawio.prop_out] = "Nice picture! Who is that person?"
            ctx[prop_face_vec] = pickle.dumps(face)
        else:
            ctx[rawio.prop_out] = f"Huh, looks interesting. But maybe send me a cute face!"

    @rs.state(
        cond=(
            sig_repeat_name.max_age(30) | prop_face_vec.changed().max_age(30)
        ) & nlp.prop_tokens.changed(),
        signal=sig_repeat_name,
        read=(nlp.prop_triples, nlp.prop_tokens, prop_face_vec),
        write=rawio.prop_out,
        emit_detached=True)
    def store_face_and_name(ctx: rs.ContextWrapper):
        tokens = ctx[nlp.prop_tokens]
        triples = ctx[nlp.prop_triples]
        if len(tokens) == 1:
            name = tokens[0]
        elif triples[0].get_object().text and triples[0].match_either_lemma(pred={"be"}):
            name = triples[0].get_object().text
        else:
            ctx["rawio:out"] = "Sorry, what was the name?"
            return rs.Emit()
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
            ctx[rawio.prop_out] = err_msg
