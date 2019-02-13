from ravestate.module import Module
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


PYROBOY_AVAILABLE = False
try:
    from pyroboy import say, listen
    PYROBOY_AVAILABLE = True
except ImportError as e:
    logger.error(f"""
--------
An exception occured during `import pyroboy`: {e}
Please make sure to have the following items installed & sourced:
1. ROS2 bouncy
2. roboy_communication
3. pyroboy
--------
    """)


DEFAULT_INTERLOC_ID = "roboyio_user"

if PYROBOY_AVAILABLE:

    with Module(name="roboyio"):

        @state(cond=s(":startup"), read="interloc:all")
        def roboy_input(ctx: ContextWrapper):

            @receptor(ctx_wrap=ctx, write="rawio:in")
            def write_console_input(ctx_input, value: str):
                ctx_input["rawio:in"] = value

            @receptor(ctx_wrap=ctx, write="interloc:all")
            def push_interloc(ctx: ContextWrapper, interlocutor_node: Node):
                if ctx.push(parentpath="interloc:all", child=PropertyBase(name=DEFAULT_INTERLOC_ID, default_value=interlocutor_node)):
                    logger.debug(f"Pushed {interlocutor_node} to interloc:all")

            @receptor(ctx_wrap=ctx, write="interloc:all")
            def pop_interloc(ctx: ContextWrapper):
                if ctx.pop(f"interloc:all:{DEFAULT_INTERLOC_ID}"):
                    logger.debug(f"Popped interloc:all:{DEFAULT_INTERLOC_ID}")

            while not ctx.shutting_down():
                input_value = listen()
                if input_value is None or not input_value.strip():
                    continue

                write_console_input(input_value)

                interloc_exists = f"interloc:all:{DEFAULT_INTERLOC_ID}" in ctx.enum("interloc:all")
                # push Node if you got a greeting
                if input_value.strip() in get_phrase_list("greeting") and not interloc_exists:
                    # set up scientio
                    sess: Session = ravestate_ontology.get_session()
                    onto: Ontology = ravestate_ontology.get_ontology()

                    # create scientio Node of type Person
                    query = Node(metatype=onto.get_type("Person"))
                    query.set_name("x")
                    interlocutor_node_list = sess.retrieve(query)
                    if not interlocutor_node_list:
                        interlocutor_node = sess.create(query)
                        logger.info(f"Created new Node in scientio session: {interlocutor_node}")
                    elif len(interlocutor_node_list) == 1:
                        interlocutor_node = interlocutor_node_list[0]
                    else:
                        logger.error(f'Found multiple Persons with name {DEFAULT_INTERLOC_ID} in scientio session. Cannot push node to interloc:all!')
                        continue

                    # push interloc-Node
                    push_interloc(interlocutor_node)

                # pop Node if you got a farewell
                elif input_value.strip() in get_phrase_list("farewells") and interloc_exists:
                    pop_interloc()


        @state(read="rawio:out")
        def roboy_output(ctx):
            say(ctx["rawio:out:changed"])
