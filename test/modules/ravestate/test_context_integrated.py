from ravestate.constraint import Signal, ConfigurableAge
from ravestate.spike import Spike
from ravestate.state import Signal, Emit
from ravestate.context import sig_startup
from ravestate.module import Module
from ravestate.testfixtures import *
from threading import Lock


def test_run_with_pressure():

    with Module(name=DEFAULT_MODULE_NAME):

        prop = Property(name=DEFAULT_PROPERTY_NAME)

        a = Signal("a")
        b = Signal("b")

        @state(cond=sig_startup, signal=a)
        def signal_a(ctx):
            return Emit()

        @state(cond=a, signal=b)
        def signal_b(ctx):
            return Emit()

        @state(cond=a, write=prop)
        def pressuring_state(ctx):
            pass

        @state(cond=a & b, write=prop)
        def specific_state(ctx):
            pass

    ctx = Context(DEFAULT_MODULE_NAME)
    ctx.emit(sig_startup)

    ctx.run_once()
    assert signal_a.wait()

    ctx.run_once()
    assert signal_b.wait()

    # make sure that pressuring_state is pressuring specific_state
    acts = ctx._state_activations(st=specific_state)
    assert any(act.is_pressured() for act in acts)

    ctx.run_once()
    assert specific_state.wait()
