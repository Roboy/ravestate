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

    @rs.state(cond=interloc.prop_all.pushed(), write=(verbaliser.prop_intent, rawio.prop_out),
              read=interloc.prop_all)
    def greeting(ctx: rs.ContextWrapper):
        interloc_node: Node = ctx[f'interloc:all:{interloc.ANON_INTERLOC_ID}']
        name = interloc_node.get_name()
        if name:
            phrase = verbaliser.get_random_phrase('greeting-with-name')
            ctx[rawio.prop_out] = phrase.format(name=name)
        else:
            ctx[verbaliser.prop_intent] = lang.intent_greeting

    @rs.state(cond=interloc.prop_all.popped() & prop_in.changed(), write=verbaliser.prop_intent)
    def farewell(ctx: rs.ContextWrapper):
        ctx[verbaliser.prop_intent] = lang.intent_farewells
