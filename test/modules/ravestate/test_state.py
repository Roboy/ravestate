import pytest
from ravestate.constraint import s

from ravestate.state import State, state
from ravestate.wrappers import ContextWrapper


@pytest.fixture
def default_signal():
    return s("test-signal")


@pytest.fixture
def default_read():
    return "myprop"


@pytest.fixture
def default_write():
    return "myprop"


@pytest.fixture
def default_triggers():
    return s(":idle")


@pytest.fixture
def default_context_wrapper(mocker):
    return mocker.Mock(name=ContextWrapper.__class__)


@pytest.fixture
def default_action(mocker):
    return mocker.Mock(name=State.__class__)


@pytest.fixture
def under_test(default_signal, default_read, default_write, default_triggers):
    return State(
        signal=default_signal,
        read=default_read,
        write=default_write,
        triggers=default_triggers,
        action=default_action
    )


def test_decorator(under_test, default_signal, default_read, default_write, default_triggers, default_action):
    @state(signal=default_signal,
           read=default_read,
           write=default_write,
           triggers=default_triggers)
    def test_state(_):
        return "Hello world!"

    assert (test_state.signal == under_test.signal)
    assert (test_state.read_props == under_test.read_props)
    assert (test_state.write_props == under_test.write_props)
    assert (test_state.triggers == under_test.triggers)
    assert (test_state(default_context_wrapper, [], {}) == "Hello world!")
    assert (isinstance(test_state.action, type(under_test.action)))


def test_decorator_illegal_trigger(under_test, default_signal, default_read, default_write, default_action):
    with pytest.raises(ValueError):
        @state(signal=default_signal,
               read=default_read,
               write=default_write,
               triggers=(s("rawio:in:changed") | s("facerec:face:changed")) & (s("sys:has-internet") | s("foo:poo")))
        def test_state(_):
            return "Hello world!"


def test_decorator_default(under_test):
    @state()
    def test_state(_):
        return "Hello world!"

    assert (test_state.signal == "")
    assert (test_state.read_props == ())
    assert (test_state.write_props == ())
    assert (test_state.triggers is None)
    assert (test_state(default_context_wrapper, [], {}) == "Hello world!")
    assert (isinstance(test_state.action, type(under_test.action)))
