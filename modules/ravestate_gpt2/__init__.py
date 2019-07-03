from threading import Lock
from typing import List

import ravestate as rs
import re
import requests

import ravestate_rawio as rawio
from ravestate_gpt2 import server

from reggol import get_logger
logger = get_logger(__name__)

# TODO server address and port configurable for running on better machine remotely
# TODO config options for gpt2

SERVER_AVAILABLE_CODE = 200
WILDTALK_SERVER_ADDRESS = "http://localhost:5100"


with rs.Module(name="gpt2"):

    logged_conversation: List[str] = []
    lock_conv_logger = Lock()

    @rs.state(cond=rs.sig_startup)
    def init_wildtalk(ctx):
        if not server_up(WILDTALK_SERVER_ADDRESS):
            server.run()  # start server

    @rs.state(read=rawio.prop_in)
    def input_logger(ctx):
        if ctx[rawio.prop_in.changed()]:
            with lock_conv_logger:
                logged_conversation.append("You: " + ctx[rawio.prop_in.changed()])
                while len(logged_conversation) > 4:
                    logged_conversation.pop(0)

    @rs.state(read=rawio.prop_out)
    def output_logger(ctx):
        if ctx[rawio.prop_out.changed()]:
            with lock_conv_logger:
                logged_conversation.append("Roboy: " + ctx[rawio.prop_out.changed()])
                while len(logged_conversation) > 4:
                    logged_conversation.pop(0)


    @rs.state(read=rawio.prop_in, write=rawio.prop_out)
    def wildtalk_state(ctx):
        """
        Wildtalk using GPT2 through a HTTP server.
        Post input and get answer through the HTTP interface.
        """
        if not server_up(WILDTALK_SERVER_ADDRESS):
            logger.error(
                "\n--------"
                "\nThe Wildtalk server does not seem to be running. Wildtalk will not work, so no fallback answers!"
                "\n--------")
            return rs.Delete(resign=True)

        prompt = ' '.join(logged_conversation) + ' Roboy: '
        params = {'prompt': prompt}
        sample = None
        while not sample:  # sample might be "fully sanitized" and empty
            response = requests.get(WILDTALK_SERVER_ADDRESS, params=params)
            response_json = response.json()
            logger.info(response_json)
            sample = sanitize_sample(response_json['response'])
        ctx[rawio.prop_out] = sample


def server_up(server_address):
    try:
        status = requests.head(server_address).status_code
    except requests.exceptions.RequestException or requests.exceptions.ConnectionError:
        status = None
    return status == SERVER_AVAILABLE_CODE


def sanitize_sample(sample: str):
    # remove groups of non-letters (incl. surrounding whitespace) e.g. ??????
    sample = re.sub(r'\s*[^\w\s]{3,}\s*', '',  sample)

    # find first occurrence of <NAME>:
    speaker_prompt = re.search(r'\S+:', sample)
    if speaker_prompt:
        # only use utterance before speaker prompt
        sample = sample[:speaker_prompt.start()]
    else:
        # cut off from the right until one of .?! is found to avoid half-sentences
        last_period = sample.rfind('.')
        last_qmark = sample.rfind('?')
        last_exmark = sample.rfind('!')
        if last_period > 0:
            sample = sample[:last_period+1]
        elif last_exmark > 0:
            sample = sample[:last_exmark+1]
        elif last_qmark > 0:
            sample = sample[:last_qmark+1]
    return sample
