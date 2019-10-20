import ravestate as rs
import ravestate_nlp as nlp
import ravestate_ontology as mem

from scientio.ontology.node import Node
from scientio.ontology.ontology import Ontology

from reggol import get_logger
logger = get_logger(__name__)

ANON_INTERLOC_ID = "anonymous_interlocutor"
ANON_INTERLOC_PATH = f"interloc:all:{ANON_INTERLOC_ID}"


with rs.Module(name="interloc", depends=(nlp.mod, mem.mod)) as mod:

    # TODO: Make interloc:all a special property type, that only accepts ScientioNodeProperty as children
    prop_all = rs.Property(name="all", allow_read=True, allow_write=False, allow_push=True, allow_pop=True)
    prop_persisted = rs.Property(name="persisted", allow_read=True, allow_write=True, allow_push=True, allow_pop=True,
                                 always_signal_changed=True)

    @rs.state(cond=nlp.sig_intent_hi, read=prop_all, write=prop_all)
    def push_interlocutor(ctx):
        interloc_exists = ANON_INTERLOC_PATH in ctx.enum(prop_all)
        if not interloc_exists:
            mem.initialized.wait()
            onto: Ontology = mem.get_ontology()
            new_interloc = Node(metatype=onto.get_type("Person"))
            new_interloc.set_name(ANON_INTERLOC_ID)
            if ctx.push(
                    parent_property_or_path=prop_all,
                    child=rs.Property(name=ANON_INTERLOC_ID, default_value=new_interloc)):
                logger.debug(f"Pushed {new_interloc} to interloc:all")

    @rs.state(cond=nlp.sig_intent_bye, read=prop_all, write=prop_all)
    def pop_interlocutor(ctx):
        interloc_exists = ANON_INTERLOC_PATH in ctx.enum(prop_all)
        if interloc_exists and ctx.pop(ANON_INTERLOC_PATH):
            logger.debug(f"Popped {ANON_INTERLOC_PATH}")
