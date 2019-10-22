import ravestate as rs

with rs.Module(name="rawio") as mod:

    prop_in = rs.Property(
        name="in",
        default_value="",
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True)

    prop_out = rs.Property(
        name="out",
        default_value="",
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True,
        wipe_on_changed=False)

    prop_pic_in = rs.Property(
        name="pic_in",
        default_value=None,
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True)


def say(ctx, what: str) -> None:
    """
    Calls an internal receptor to write a value to `rawio.prop_in`.
     Use this function in any IO module that has a state with a long-running loop.
    """
    @rs.receptor(ctx_wrap=ctx, write=prop_in)
    def write_input(ctx_input, value: str):
        ctx_input[prop_in] = value
    write_input(what)
