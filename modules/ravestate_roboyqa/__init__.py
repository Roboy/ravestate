from ravestate import registry
from ravestate.state import state
from ravestate.constraint import s

@state(triggers=s(":startup"), write="rawio:out")
def hello_world_roboyqa(ctx):
    ctx["rawio:out"] = "Ask me something about myself!"

@state(read="nlp:triples", write="rawio:out")
def roboyqa(ctx):
    question_triple = ctx["nlp:triples"]
    ctx["rawio:out"] = question_triple

registry.register(
    name="roboyqa",
    states=(hello_world_roboyqa, roboyqa)
)