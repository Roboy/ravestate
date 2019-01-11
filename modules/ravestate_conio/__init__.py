from h5py.h5p import DEFAULT

from ravestate import registry
from ravestate.constraint import s
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate.receptor import receptor
from ravestate.wrappers import ContextWrapper

import ravestate_rawio
import ravestate_interloc
from ravestate_verbaliser.verbaliser import get_phrase_list
import ravestate_phrases_basic_en
import ravestate_ontology

from scientio.ontology.node import Node
from scientio.session import Session
from scientio.ontology.ontology import Ontology

from reggol import get_logger
logger = get_logger(__name__)

DEFAULT_INTERLOC_ID = "terminal_user"


@state(cond=s(":startup"), read="interloc:all")
def console_input(ctx: ContextWrapper):

    @receptor(ctx_wrap=ctx, write="rawio:in")
    def write_console_input(ctx_input, value: str):
        ctx_input["rawio:in"] = value

    @receptor(ctx_wrap=ctx, write="interloc:all")
    def push_console_interloc(ctx: ContextWrapper, console_node: Node):
        if ctx.push(parentpath="interloc:all", child=PropertyBase(name=DEFAULT_INTERLOC_ID, default_value=console_node)):
            logger.debug(f"Pushed {console_node} to interloc:all")

    @receptor(ctx_wrap=ctx, write="interloc:all")
    def pop_console_interloc(ctx: ContextWrapper):
        if ctx.pop(f"interloc:all:{DEFAULT_INTERLOC_ID}"):
            logger.debug(f"Popped interloc:all:{DEFAULT_INTERLOC_ID}")

    while not ctx.shutting_down():
        input_value = input("> ")
        write_console_input(input_value)

        console_interloc_exists = f"interloc:all:{DEFAULT_INTERLOC_ID}" in ctx.enum("interloc:all")
        # push Node if you got a greeting
        if input_value.strip() in get_phrase_list("greeting") and not console_interloc_exists:
            # set up scientio
            sess: Session = ravestate_ontology.get_session()
            onto: Ontology = ravestate_ontology.get_ontology()

            # create scientio Node of type Person
            query = Node(metatype=onto.get_type("Person"))
            query.set_name("x")
            console_node_list = sess.retrieve(query)
            if not console_node_list:
                console_node = sess.create(query)
                logger.info(f"Created new Node in scientio session: {console_node}")
            elif len(console_node_list) == 1:
                console_node = console_node_list[0]
            else:
                logger.error(f'Found multiple Persons with name {DEFAULT_INTERLOC_ID} in scientio session. Cannot push node to interloc:all!')
                continue

            # push interloc-Node
            push_console_interloc(console_node)

        # pop Node if you got a farewell
        elif input_value.strip() in get_phrase_list("farewells") and console_interloc_exists:
            pop_console_interloc()


@state(read="rawio:out")
def console_output(ctx):
    print(ctx["rawio:out"])


registry.register(
    name="consoleio",
    states=(console_input, console_output)
)
