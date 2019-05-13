import ravestate as rs
import requests

import ravestate_idle as idle
import ravestate_rawio as rawio
import ravestate_nlp as nlp
import ravestate_verbaliser as verbaliser

from reggol import get_logger
logger = get_logger(__name__)

DRQA_SERVER_ADDRESS: str = "drqa_server_address"
ROBOY_ANSWER_SANITY: str = "roboy_answer_sanity"
CONFIG = {
    DRQA_SERVER_ADDRESS: "http://localhost:5000",
    ROBOY_ANSWER_SANITY: 5000
}
SERVER_AVAILABLE_CODE = 200


with rs.Module(name="genqa", config=CONFIG):

    @rs.state(cond=rs.sig_startup)
    def hello_world_genqa(ctx):
        server = ctx.conf(key=DRQA_SERVER_ADDRESS)
        if not server:
            logger.error('Server address is not set. Shutting down GenQA.')
            return rs.Delete()
        if not server_up(server):
            return rs.Delete()

    @rs.state(cond=idle.sig_bored, write=rawio.prop_out, weight=1.15, cooldown=30.)
    def prompt(ctx):
        ctx[rawio.prop_out] = verbaliser.get_random_phrase("question-answering-prompt")

    @rs.state(cond=nlp.sig_is_question, read=rawio.prop_in, write=rawio.prop_out)
    def drqa_module(ctx):
        """
        general question answering using DrQA through a HTTP server
        connection check to server
        post input question and get DrQA answer through the HTTP interface
        depending on the answer score the answer is introduced as sane or insane/unsure :)
        """
        server = ctx.conf(key=DRQA_SERVER_ADDRESS)
        if not server_up(server):
            return rs.Delete(resign=True)
        params = {'question': str(ctx[rawio.prop_in]).lower()}
        response = requests.get(server, params=params)
        response_json = response.json()
        certainty = response_json["answers"][0]["span_score"]
        # sane answer
        if certainty > ctx.conf(key=ROBOY_ANSWER_SANITY):
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("question-answering-starting-phrases") + " " + \
                               response_json["answers"][0]["span"]
        # insane/unsure answer
        else:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("unsure-question-answering-phrases") \
                               % response_json["answers"][0]["span"] \
                               + "\n" + "Maybe I can find out more if your rephrase the question for me."


def server_up(server):
    status = None
    try:
        status = requests.head(server).status_code
    except requests.exceptions.RequestException or requests.exceptions.ConnectionError:
        logger.error(
            "\n--------"
            "\nThe DrQA server does not seem to be running. GenQA will not work, so no asking general questions!"
            "\nTo set up DrQA, follow the instructions here: https://github.com/roboy/drqa."
            "\n--------")
    return status == SERVER_AVAILABLE_CODE

