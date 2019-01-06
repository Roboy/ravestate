from ravestate import registry
from ravestate.state import state, Delete
from ravestate_verbaliser import verbaliser
from ravestate.constraint import s

import ravestate_rawio
import ravestate_phrases_basic_en

import requests
from reggol import get_logger

logger = get_logger(__name__)

DRQA_SERVER_ADDRESS: str = "drqa_server_address"
best_answer = 0


@state(cond=s(":startup"), write="rawio:out")
def hello_world_genqa(ctx):
    ctx["rawio:out"] = "Ask me anything!"
    server = ctx.conf(key=DRQA_SERVER_ADDRESS)
    if not server:
        logger.error('Server address is not set. Shutting down GenQA.')
        return Delete()


@state(read="rawio:in", write="rawio:out")
def drqa_module(ctx):
    params = {'question': ctx["rawio:in"]}
    answer_phrase = verbaliser.get_random_phrase("question-answering-starting-phrases")
    server = ctx.conf(key=DRQA_SERVER_ADDRESS)
    if not server:
        logger.error('Server address is not set. Shutting down GenQA.')
        return Delete()
    response = requests.get(server, params=params)
    response_json = response.json()
    ctx["rawio:out"] = answer_phrase + " " + response_json["answers"][best_answer]["span"]


registry.register(
    name="genqa",
    states=(hello_world_genqa, drqa_module),
    config={
        DRQA_SERVER_ADDRESS: "http://localhost:5000"
    }
)
