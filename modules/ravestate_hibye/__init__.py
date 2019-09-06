import ravestate as rs
import ravestate_verbaliser as verbaliser
import ravestate_phrases_basic_en as lang
import ravestate_interloc as interloc
import ravestate_rawio as rawio
from ravestate_rawio import prop_in
from scientio.ontology.node import Node
from os.path import realpath, dirname, join

verbaliser.add_folder(join(dirname(realpath(__file__)), "en"))

with rs.Module(name="hibye"):

    @rs.state(cond=interloc.prop_all.pushed(), write=rawio.prop_out,
              read=interloc.prop_all)
    def greeting(ctx: rs.ContextWrapper):
        print('Greeting a node: ')
        pushed_node_path: str = ctx[interloc.prop_all.pushed()]
        interloc_node: Node = ctx[pushed_node_path]
        # interloc_node: Node = ctx[f'interloc:all:{interloc.ANON_INTERLOC_ID}']
        if interloc_node and interloc_node.get_id() >= 0:
            phrase = verbaliser.get_random_phrase('greeting-with-name')
            ctx[rawio.prop_out] = phrase.format(name=interloc_node.get_name())
            print('here1')
        else:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase(lang.intent_greeting)
            print('here2')

    @rs.state(cond=interloc.prop_all.popped(), write=verbaliser.prop_intent)
    def farewell(ctx: rs.ContextWrapper):
        ctx[verbaliser.prop_intent] = lang.intent_farewells
