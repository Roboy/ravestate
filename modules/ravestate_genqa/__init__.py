from ravestate import registry
from ravestate.state import state
import ravestate_rawio
from ravestate.constraint import s

import logging
import requests

@state(triggers=s(":startup"), write="rawio:out")
def hello_world(ctx):
    ctx["rawio:out"] = "Ask me anything!"

@state(read="rawio:in", write="rawio:out")
def drqa_module(ctx):
    params = {'question': ctx["rawio:in"]}
    response = requests.get('http://localhost',params=params)
    ctx["rawio:out"] = response.json()

registry.register(
    name="genqa",
    states=(hello_world, drqa_module)
)
