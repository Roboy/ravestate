import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_persqa as persqa
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


def test_unknown_person():
    last_output = ""

    with rs.Module(name="visionio_test"):

        @rs.state(read=rawio.prop_out)
        def raw_out(ctx: rs.ContextWrapper):
            nonlocal last_output
            last_output = ctx[rawio.prop_out]
            logger.info(f"Output: {ctx[rawio.prop_out]}")

        ctx = rs.Context(
            "rawio",
            "ontology",
            "verbaliser",
            "idle",
            "interloc",
            "nlp",
            "persqa",
            "hibye",
            "visionio",
            "visionio_test",
            "-d", "ontology", "neo4j_pw", "test"
        )

    @rs.receptor(ctx_wrap=ctx, write=visionio.prop_subscribe_faces)
    def unknown_person_approaches(ctx: rs.ContextWrapper):
        faces = Faces()
        faces.confidence = [0.5]
        faces.ids = [42]

        facial_features = FacialFeatures()
        facial_features.ff = np.zeros(128)
        faces.face_encodings = [facial_features]

        ctx[visionio.prop_subscribe_faces] = faces

    ctx.emit(rs.sig_startup)
    ctx.run_once()
    assert mem.initialized.wait()

    # Vision io is started
    assert visionio.reset.wait()

    unknown_person_approaches()

    # Wait until greeted
    counter = 0
    while not raw_out.wait(.1) and counter < 100:
        ctx.run_once()
        counter += 1
    assert last_output in verbaliser.get_phrase_list("greeting")

    assert visionio.recognize_faces.wait(0)

    # Unfortunately needed until Context adopts Properties as clones.
    interloc.prop_all.children.clear()


if __name__ == "__main__":
    set_default_loglevel("DEBUG")
    test_unknown_person()
    exit()
