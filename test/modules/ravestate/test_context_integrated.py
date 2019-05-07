from ravestate.constraint import Signal, ConfigurableAge
from ravestate.spike import Spike
from ravestate.state import s, Emit
from ravestate.context import startup
from ravestate.module import Module
from ravestate.testfixtures import *
from threading import Lock


def test_run_with_pressure():

    with Module(name=DEFAULT_MODULE_NAME):

        Property(name=DEFAULT_PROPERTY_NAME)

        @state(cond=startup(), signal_name="a")
        def signal_a(ctx):
            return Emit()

        @state(cond=s(f"{DEFAULT_MODULE_NAME}:a"), signal_name="b")
        def signal_b(ctx):
            return Emit()

        @state(cond=s(f"{DEFAULT_MODULE_NAME}:a"), write=DEFAULT_PROPERTY_ID)
        def pressuring_state(ctx):
            pass

        @state(cond=s(f"{DEFAULT_MODULE_NAME}:a") & s(f"{DEFAULT_MODULE_NAME}:b"), write=DEFAULT_PROPERTY_ID)
        def specific_state(ctx):
            pass

    ctx = Context(DEFAULT_MODULE_NAME)
    ctx.emit(startup())

    ctx.run_once()
    assert signal_a.wait()

    ctx.run_once()
    assert signal_b.wait()

    # make sure that pressuring_state is pressuring specific_state
    acts = ctx._state_activations(st=specific_state)
    assert any(act.is_pressured() for act in acts)

    ctx.run_once()
    assert specific_state.wait()
