import ravestate as rs

import ravestate_interloc as interloc

from reggol import get_logger
logger = get_logger(__name__)

IMPATIENCE_THRESHOLD_CONFIG_KEY = "impatience_threshold"
BORED_THRESHOLD_CONFIG_KEY = "bored_threshold"
CONFIG = {
    # duration in seconds how long ":pressure" should be true before getting impatient
    IMPATIENCE_THRESHOLD_CONFIG_KEY:  1.0,
    BORED_THRESHOLD_CONFIG_KEY: 1.5
}

with rs.Module(name="idle", config=CONFIG, depends=(interloc.mod,)) as mod:

    sig_impatient = rs.Signal(name="impatient")
    sig_bored = rs.Signal(name="bored")
    sig_bored_by_user = rs.Signal(name="bored-by-user")

    @rs.state(
        cond=rs.prop_activity.changed().min_age(rs.ConfigurableAge(key=BORED_THRESHOLD_CONFIG_KEY)).max_age(-1),
        read=rs.prop_activity,
        signal=sig_bored)
    def am_i_bored(ctx: rs.ContextWrapper):
        """
        Emits idle:bored signal if no states are currently partially fulfilled
        """
        if ctx[rs.prop_activity] == 0:
            return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_bored,
        read=interloc.prop_all,
        signal=sig_bored_by_user)
    def am_i_bored_by_user(ctx: rs.ContextWrapper):
        """
        Emits idle:bored-by-user idle:bored as emitted and there is a present interlocutor.
        """
        if any(ctx.enum(interloc.prop_all)):
            return rs.Emit(wipe=True)

    @rs.state(
        cond=rs.prop_pressure.true().min_age(rs.ConfigurableAge(key=IMPATIENCE_THRESHOLD_CONFIG_KEY)).max_age(-1.),
        signal=sig_impatient)
    def am_i_impatient(ctx: rs.ContextWrapper):
        return rs.Emit(wipe=True)
