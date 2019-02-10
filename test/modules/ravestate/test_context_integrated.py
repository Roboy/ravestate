from ravestate.constraint import Signal, ConfigurableAge
from ravestate.spike import Spike
from ravestate.module import Module
from ravestate.testfixtures import *
from threading import Lock


def test_run(
        context_with_property_fixture: Context,
        state_signal_a_fixture: State,
        state_signal_b_fixture: State,
        state_signal_c_fixture: State,
        state_signal_d_fixture: State):

    waiter = Lock()
    waiter.acquire()

    @state(write=(DEFAULT_PROPERTY_ID,), read=(DEFAULT_PROPERTY_ID,))
    def state_mock_fn(ctx):
        waiter.release()
    state_mock_fn.module_name = "test"

    context_with_property_fixture.add_state(st=state_mock_fn)
    context_with_property_fixture._run_once()

    # Create a property wrapper to trigger a changed signal
    wrap = PropertyWrapper(
        prop=context_with_property_fixture[DEFAULT_PROPERTY_ID],
        ctx=context_with_property_fixture,
        allow_read=False,
        allow_write=True)
    wrap.set("test")
    del wrap  # delete to release lock on property

    context_with_property_fixture._run_once()
    assert waiter.acquire(blocking=True, timeout=3.)
