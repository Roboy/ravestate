import ravestate as rs
import requests

import ravestate_rawio as rawio
from ravestate_wildtalk import server

from reggol import get_logger
logger = get_logger(__name__)

MODEL_KEY = "model"
SERVER_ADDRESS_KEY = "server_address"
SERVER_PORT_KEY = "server_port"
TEMPERATURE_KEY = "temperature"
MAX_LENGTH_KEY = "max_length"
TOP_K_KEY = "top_k"
TOP_P_KEY = "top_p"
MAX_HISTORY_KEY = "max_history"

AVAILABLE_MODELS = ["convai_gpt", "gpt2", "parlai"]

CONFIG = {
    MODEL_KEY: AVAILABLE_MODELS[0],  # one of "convai_gpt", "gpt2", "parlai"
    SERVER_ADDRESS_KEY: "http://localhost",  # can be changed if server is running on its own on a separate machine
    SERVER_PORT_KEY: 5100,
    TEMPERATURE_KEY: 0.7,  # convai_gpt, gpt2: higher value -> more variation in output
    MAX_LENGTH_KEY: 20,  # convai_gpt, gpt2: maximal length of generated output
    TOP_K_KEY: 0,  # convai_gpt, gpt2: <=0: no filtering, >0: keep only top k tokens with highest probability.
    TOP_P_KEY: 0.9,  # convai_gpt: <=0.0 no filtering, >0.0: keep smallest subset whose total probability mass >= top_p
    MAX_HISTORY_KEY: 4,  # convai_gpt: maximal number of previous dialog turns to be used for output generation

}

server_started = False

with rs.Module(name="wildtalk", config=CONFIG):
    @rs.state(cond=rs.sig_startup)
    def init_wildtalk(ctx):
        server_address = f"{ctx.conf(key=SERVER_ADDRESS_KEY)}:{ctx.conf(key=SERVER_PORT_KEY)}"
        if not server_up(server_address):
            model = ctx.conf(key=MODEL_KEY)
            if model not in AVAILABLE_MODELS:
                logger.error(
                    f"{ctx.conf(key=MODEL_KEY)} is not an available wildtalk model. Falling back to using {AVAILABLE_MODELS[0]}")
                model = AVAILABLE_MODELS[0]
            global server_started
            server_started = True
            logger.info("Starting up wildtalk server")
            server.run(port=ctx.conf(key=SERVER_PORT_KEY), model=model)  # start server
            server_started = False

    @rs.state(cond=rawio.prop_in.changed().max_age(-1), read=rawio.prop_in, write=rawio.prop_out)
    def wildtalk_state(ctx):
        """
        Wildtalk using a HTTP server.
        Post input and get answer through the HTTP interface.
        """
        server_address = f"{ctx.conf(key=SERVER_ADDRESS_KEY)}:{ctx.conf(key=SERVER_PORT_KEY)}"
        if not server_up(server_address):
            global server_started
            if server_started:
                # don't resign if server started
                logger.info(
                    "\n--------"
                    "\nThe Wildtalk server is still starting up. It will not react to this input."
                    "\n--------")
                return
            else:
                logger.error(
                    "\n--------"
                    "\nThe Wildtalk server does not seem to be running. Wildtalk will not work, so no fallback answers!"
                    "\n--------")
                return rs.Delete(resign=True)

        params = {'prompt': ctx[rawio.prop_in.changed()]}
        for key in [TEMPERATURE_KEY, MAX_LENGTH_KEY, TOP_K_KEY, TOP_P_KEY, MAX_HISTORY_KEY]:
            params[key] = ctx.conf(key=key)
        sample = None
        while not sample:  # sample might be "fully sanitized" and empty
            response = requests.get(server_address, params=params)
            response_json = response.json()
            logger.info(response_json)
            sample = response_json['response'].strip()
        ctx[rawio.prop_out] = sample


def server_up(server_address):
    try:
        status = requests.head(server_address).status_code
    except requests.exceptions.RequestException or requests.exceptions.ConnectionError:
        status = None
    return status == 200
