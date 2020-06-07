import ravestate as rs
import ravestate_emotion as emo
import ravestate_rawio as rawio

import random

SHY_EMOJIS = ["\U0000263A"]
SURPRISE_EMOJIS = ["\U0001F914"]
HAPPY_EMOJIS = ["\U0001F60A", "\U0001F603"]
AFFECTIONATE_EMOJIS = ["\U0001F618", "\U0001F970"]


with rs.Module(name="emoji", depends=(emo.mod, rawio.mod)) as mod:

    @rs.state(cond=emo.sig_shy.detached(), write=rawio.prop_out)
    def show_shy(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = random.choice(SHY_EMOJIS)


    @rs.state(cond=emo.sig_surprise.detached(), write=rawio.prop_out)
    def show_surprise(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = random.choice(SURPRISE_EMOJIS)


    @rs.state(cond=emo.sig_happy.detached(), write=rawio.prop_out)
    def show_happy(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = random.choice(HAPPY_EMOJIS)


    @rs.state(cond=emo.sig_affectionate.detached(), write=rawio.prop_out)
    def show_affectionate(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = random.choice(AFFECTIONATE_EMOJIS)
