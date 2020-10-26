import ravestate as rs
import ravestate_rawio as rawio
import requests
import os

from .bad_words import contains_bad_word

from reggol import get_logger
logger = get_logger(__name__)

# -- Prompt setup

PREFIX_ROBOY = "roboy: "
PREFIX_HUMAN = "human: "
PROMPT = f"""
the following is a conversation between a human and a young robot called "roboy". he is curious, cheeky and nerdy:
"""
# {PROMPT_CONVERSATION_HUMAN} How are you?
# {PROMPT_CONVERSATION_ROBOY} I'm doing great! There's nothing in the world I enjoy more than hanging out with interesting people.
# {PROMPT_CONVERSATION_HUMAN} What is your name?
# {PROMPT_CONVERSATION_ROBOY} My name is Roboy. I'm a robot boy you know!
# {PROMPT_CONVERSATION_HUMAN} Where are you from?
# {PROMPT_CONVERSATION_ROBOY} I am from Munich, but I can speak any language.

# -- Configuration

GPT3_API_KEY = "key"
ROBOY_PROMPT_PREFIX_KEY = "prompt-roboy-prefix"
HUMAN_PROMPT_PREFIX_KEY = "prompt-human-prefix"
PROMPT_KEY = "prompt"
TEMPERATURE_KEY = "temperature"
MAX_TOKENS_KEY = "max-tokens"
MIN_PROB_KEY = "min-probability"
STOP_INDICATORS_KEY = "stop-indicators"
CENSOR_KEY = "censor"
CENSOR_RESPONSE_KEY = "censor-response"
CONFIG = {
    GPT3_API_KEY: "",
    PROMPT_KEY: PROMPT,
    HUMAN_PROMPT_PREFIX_KEY: PREFIX_HUMAN,
    ROBOY_PROMPT_PREFIX_KEY: PREFIX_ROBOY,
    TEMPERATURE_KEY: 1.1,
    MAX_TOKENS_KEY: 64,
    MIN_PROB_KEY: 0.5,
    STOP_INDICATORS_KEY: ["\n"],
    CENSOR_KEY: True,
    CENSOR_RESPONSE_KEY: "i'm not sure. can we can talk about something else?"
}


with rs.Module(name="gpt3", config=CONFIG, depends=(rawio.mod,)) as mod:

    history = []

    def append_to_history(who: str, what: str, ignored_response: str=None):
        if what.startswith("Watch your conversation on Raveboard") or \
                ignored_response and what.startswith(ignored_response):
            return
        combined = who + what
        if not history or history[-1] != combined:
            history.append(combined)

    @rs.state(cond=rawio.prop_in.changed().detached(), read=rawio.prop_in, boring=True)
    def input_seen(ctx):
        append_to_history(ctx.conf(key=HUMAN_PROMPT_PREFIX_KEY), ctx[rawio.prop_in], )

    @rs.state(cond=rawio.prop_out.changed().detached(), read=rawio.prop_out, boring=True)
    def output_seen(ctx):
        append_to_history(
            ctx.conf(key=ROBOY_PROMPT_PREFIX_KEY),
            ctx[rawio.prop_out],
            ctx.conf(key=CENSOR_RESPONSE_KEY))

    @rs.state(cond=rawio.prop_in.changed().max_age(-1), read=(rawio.prop_in,), write=(rawio.prop_out,))
    def gpt3(ctx: rs.ContextWrapper):
        global history
        append_to_history(ctx.conf(key=HUMAN_PROMPT_PREFIX_KEY), ctx[rawio.prop_in])
        prompt = ctx.conf(key=PROMPT_KEY) + "\n".join(history + ctx.conf(key=ROBOY_PROMPT_PREFIX_KEY))
        logger.info(prompt)
        result = requests.post("https://api.openai.com/v1/engines/davinci/completions", json={
            "prompt": prompt,
            "temperature": ctx.conf(key=TEMPERATURE_KEY),
            "max_tokens": ctx.conf(key=MAX_TOKENS_KEY),
            "top_p": ctx.conf(key=MIN_PROB_KEY),
            "stop": ctx.conf(key=STOP_INDICATORS_KEY)
        }, headers={
            "Authorization": f"Bearer {ctx.conf(key=GPT3_API_KEY)}"
        }).json()
        response = result["choices"][0]["text"]
        logger.info(f"GPT-3 response: {response}")
        response_is_censored = ctx.conf(key=CENSOR_KEY) and contains_bad_word(response)
        if not response or response_is_censored:
            logger.info("The response is empty or contains a bad word.")
            # remove the offending user utterance from history
            history = history[:-1]
            ctx[rawio.prop_out] = ctx.conf(key=CENSOR_RESPONSE_KEY)
        else:
            ctx[rawio.prop_out] = response
