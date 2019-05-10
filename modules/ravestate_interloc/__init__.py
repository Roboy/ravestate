import ravestate as rs
import ravestate_rawio as rawio
import ravestate_verbaliser as verbaliser
import ravestate_phrases_basic_en as lang
import ravestate_ontology as mem

from scientio.ontology.node import Node
from scientio.session import Session
from scientio.ontology.ontology import Ontology

from reggol import get_logger
logger = get_logger(__name__)


with rs.Module(name="interloc"):

    # TODO: Make interloc:all a special property type, that only accepts ScientioNodeProperty as children
    prop_all = rs.Property(name="all", allow_read=True, allow_write=False, allow_push=True, allow_pop=True)


def handle_single_interlocutor_input(ctx: rs.ContextWrapper, input_value: str, id="anonymous_interlocutor") -> None:
    """
    Forwards input to `rawio:in` and manages creation/deletion of a singleton
     interlocutor. A new interlocutor node is pushed, when the input is a greeting,
     and there is no interlocutor present. The interlocutor is popped,
     if the input is a farewell, and there is an interlocutor present.

    * `ctx`: Context Wrapper of the calling state. Must have read permissions
     for `interloc:all`.

    * `input_value`: The string input value that should be written to `rawio:in`.

    * `id`: Name of the interlocutor context property, and initial name for
     the interlocutor's Neo4j node (until a proper name is set by persqa).
    """

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
    def write_input(ctx_input, value: str):
        ctx_input[rawio.prop_in] = value

    @rs.receptor(ctx_wrap=ctx, write=prop_all)
    def push_interloc(ctx: rs.ContextWrapper, interlocutor_node: Node):
        if ctx.push(
                parent_property_or_path=prop_all,
                child=rs.Property(name=id, default_value=interlocutor_node)):
            logger.debug(f"Pushed {interlocutor_node} to interloc:all")

    @rs.receptor(ctx_wrap=ctx, write=prop_all)
    def pop_interloc(ctx: rs.ContextWrapper):
        if ctx.pop(f"interloc:all:{id}"):
            logger.debug(f"Popped interloc:all:{id}")

    write_input(input_value)

    interloc_exists = f"interloc:all:{id}" in ctx.enum(prop_all)

    # push Node if you got a greeting
    if input_value.strip() in verbaliser.get_phrase_list(lang.intent_greeting) and not interloc_exists:
        # set up scientio
        mem.initialized.wait()
        onto: Ontology = mem.get_ontology()

        # create scientio Node of type Person
        new_interloc = Node(metatype=onto.get_type("Person"))
        new_interloc.set_name(id)
        push_interloc(new_interloc)

    # pop Node if you got a farewell
    elif input_value.strip() in verbaliser.get_phrase_list("farewells") and interloc_exists:
        pop_interloc()
