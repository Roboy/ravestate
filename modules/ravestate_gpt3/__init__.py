import ravestate as rs
import ravestate_rawio as rawio
import requests

from reggol import get_logger
logger = get_logger(__name__)

GPT3_API_KEY = "key"
CONFIG = {
    GPT3_API_KEY: ""
}


PROMPT_CONVERSATION_ROBOY = "Roboy: "
PROMPT_CONVERSATION_HUMAN = "Human: "
PROMPT = f"""
The following is a conversation between a human and a young Robot called "Roboy". He is curious, cheeky and nerdy:
"""

#
# {PROMPT_CONVERSATION_HUMAN} How are you?
# {PROMPT_CONVERSATION_ROBOY} I'm doing great! There's nothing in the world I enjoy more than hanging out with interesting people.
# {PROMPT_CONVERSATION_HUMAN} What is your name?
# {PROMPT_CONVERSATION_ROBOY} My name is Roboy. I'm a robot boy you know!
# {PROMPT_CONVERSATION_HUMAN} Where are you from?
# {PROMPT_CONVERSATION_ROBOY} I am from Munich, but I can speak any language.


with rs.Module(name="gpt3", config=CONFIG, depends=(rawio.mod,)) as mod:

    history = []

    def append_to_history(who: str, what: str):
        if what.startswith("Watch your conversation on Raveboard"):
            return
        combined = who + what
        if not history or history[-1] != combined:
            history.append(combined)

    @rs.state(cond=rawio.prop_in.changed().detached(), read=rawio.prop_in, boring=True)
    def input_seen(ctx):
        append_to_history(PROMPT_CONVERSATION_HUMAN, ctx[rawio.prop_in])

    @rs.state(cond=rawio.prop_out.changed().detached(), read=rawio.prop_out, boring=True)
    def output_seen(ctx):
        append_to_history(PROMPT_CONVERSATION_ROBOY, ctx[rawio.prop_out])

    @rs.state(cond=rawio.prop_in.changed().max_age(-1), read=(rawio.prop_in,), write=(rawio.prop_out,))
    def gpt3(ctx: rs.ContextWrapper):
        append_to_history(PROMPT_CONVERSATION_HUMAN, ctx[rawio.prop_in])
        prompt = PROMPT + "\n".join(history + [PROMPT_CONVERSATION_ROBOY])
        logger.info(prompt)
        result = requests.post("https://api.openai.com/v1/engines/davinci/completions", json={
            "prompt": prompt,
            "temperature": 1.2,
            "max_tokens": 64,
            "top_p": 0.5,
            "stop": ["\n"]
        }, headers={
            "Authorization": f"Bearer {ctx.conf(key=GPT3_API_KEY)}"
        }).json()
        logger.info(result)
        ctx[rawio.prop_out] = result["choices"][0]["text"]
