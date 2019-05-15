import ravestate as rs

with rs.Module(name="rawio"):

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
