import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
import ravestate_ontology as mem
import ravestate_verbaliser as verbaliser
import ravestate_visionio as visionio
import ravestate_hibye as hibye
from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node
import numpy as np
import rospy
import random
from roboy_cognition_msgs.msg import Faces, FacialFeatures

from reggol import get_logger, set_default_loglevel
logger = get_logger(__name__)


def test_known_person():
    last_output = ""

    with rs.Module(name="visionio_test"):

        @rs.state(read=rawio.prop_out)
        def raw_out(ctx: rs.ContextWrapper):
            nonlocal last_output
            last_output = ctx[rawio.prop_out]
            logger.info(f"Output: {ctx[rawio.prop_out]}")

    # Unfortunately needed until Context adopts Properties as clones.
    interloc.prop_all.children.clear()
    ctx = rs.Context(
        "rawio",
        "ontology",
        "verbaliser",
        "idle",
        "interloc",
        "nlp",
        "hibye",
        "visionio",
        "visionio_test",
        "-d", "ontology", "neo4j_pw", "test"
    )

    def register_dummy_known_person_to_db():
        onto: Ontology = mem.get_ontology()
        sess: Session = mem.get_session()
        person_node = Node(metatype=onto.get_type("Person"))
        person_node.set_properties({'name': 'visionio_test_person'})
        person_node = sess.create(person_node)
        return person_node

    def delete_dummy_people():
        onto: Ontology = mem.get_ontology()
        sess: Session = mem.get_session()
        person_node = Node(metatype=onto.get_type("Person"))
        person_node.set_properties({'name': 'visionio_test_person'})
        # TODO: Delete method is not working!
        sess.delete(person_node)

    @rs.receptor(ctx_wrap=ctx, write=visionio.prop_subscribe_faces)
    def known_person_approaches(ctx: rs.ContextWrapper):
        person = register_dummy_known_person_to_db()

        faces = Faces()
        faces.confidence = [0.85]
        faces.ids = [person.get_id()]

        facial_features = FacialFeatures()
        facial_features.ff = np.zeros(128)
        faces.face_encodings = [facial_features]

        ctx[visionio.prop_subscribe_faces] = faces

    ctx.emit(rs.sig_startup)
    ctx.run_once()
    assert mem.initialized.wait()

    # Vision io is started
    assert visionio.reset.wait()

    known_person_approaches()

    # Wait until greeted
    counter = 0
    while not raw_out.wait(.1) and counter < 100:
        ctx.run_once()
        counter += 1
    greeting_phrases = [phrase.replace('{name}', 'visionio_test_person') for phrase in verbaliser.get_phrase_list("greeting-with-name")]
    assert last_output in greeting_phrases

    assert visionio.recognize_faces.wait(0)

    delete_dummy_people()
    # Unfortunately needed until Context adopts Properties as clones.
    interloc.prop_all.children.clear()


if __name__ == "__main__":
    set_default_loglevel("DEBUG")
    test_known_person()
    exit()
