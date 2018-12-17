from ravestate import registry
from ravestate.state import state
import ravestate_rawio
from ravestate_verbaliser import verbaliser
import ravestate_phrases_basic_en
from ravestate.constraint import s

import logging
import requests

drqa_server_url = 'http://localhost'
best_answer = 0

@state(triggers=s(":startup"), write="rawio:out")
def hello_world_genqa(ctx):
    ctx["rawio:out"] = "Ask me anything!"

@state(read="rawio:in", write=("rawio:out"))
def drqa_module(ctx):
    params = {'question': ctx["rawio:in"]}
    answer_phrase = verbaliser.get_random_phrase("question-answering-starting-phrases")
    response = requests.get(drqa_server_url, params=params)
    response_json = response.json()
    ctx["rawio:out"] = answer_phrase + " " + response_json["answers"][best_answer]["span"]

registry.register(
    name="genqa",
    states=(hello_world_genqa, drqa_module)
)
