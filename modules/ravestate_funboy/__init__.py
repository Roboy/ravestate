import ravestate as rs
import ravestate_nlp as nlp
import ravestate_verbaliser as verbaliser
import ravestate_idle as idle
import ravestate_rawio as rawio
import ravestate_ontology

from os.path import realpath, dirname, join

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node

from reggol import get_logger
logger = get_logger(__name__)

verbaliser.add_file(join(dirname(realpath(__file__)), "jokes_data.yml"))

with rs.Module(
        name="funboy",
        config={},
        depends=(verbaliser.mod, rawio.mod)) as mod:

    @rs.state(cond=rawio.prop_in.changed().max_age(-1), read=rawio.prop_in, write=rawio.prop_out)
    def funboy(ctx):
        session: Session = ravestate_ontology.get_session()
        ontology: Ontology = ravestate_ontology.get_ontology()

        ctx[rawio.prop_out] = verbaliser.get_random_phrase("gpt2-jokes")
