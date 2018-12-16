from ravestate import registry
from ravestate.state import state
import ravestate_rawio
from ravestate_verbaliser import verbaliser
import ravestate_phrases_basic_en
from ravestate.constraint import s

import logging
import requests

@state(triggers=s(":startup"), write="rawio:out")
def hello_world(ctx):
    ctx["rawio:out"] = "Ask me anything!"

@state(read="rawio:in", write=("rawio:out"))
def drqa_module(ctx):
    params = {'question': ctx["rawio:in"]}
    #ctx["verbaliser:intent"] = "question-answering-starting-phrases"
    answer_phrase = verbaliser.get_random_phrase("question-answering-starting-phrases")
    response = requests.get('http://localhost',params=params)
    response_json = response.json()
    ctx["rawio:out"] = answer_phrase + " " + response_json["answers"][0]["span"]

registry.register(
    name="genqa",
    states=(hello_world, drqa_module)
)
