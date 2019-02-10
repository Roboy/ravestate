import pytest

from ravestate.spike import Spike
from reggol import strip_prefix
from testfixtures import LogCapture

from ravestate.constraint import s
from ravestate.context import Context
from ravestate.property import PropertyBase
from ravestate.state import State, state
from ravestate.wrappers import PropertyWrapper, ContextWrapper
from ravestate.activation import Activation

DEFAULT_MODULE_NAME = 'module'
DEFAULT_PROPERTY_NAME = 'property'
DEFAULT_PROPERTY_ID = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}"
DEFAULT_PROPERTY_VALUE = 'Kruder'
DEFAULT_PROPERTY_CHANGED = f"{DEFAULT_PROPERTY_ID}:changed"
NEW_PROPERTY_VALUE = 'Dorfmeister'


@pytest.fixture
def state_fixture(mocker):
    @state(write=(DEFAULT_PROPERTY_ID,), read=(DEFAULT_PROPERTY_ID,))
    def state_mock_fn(ctx):
        ctx[DEFAULT_PROPERTY_ID] = "test"
    state_mock_fn.module_name = DEFAULT_MODULE_NAME
    return state_mock_fn


@pytest.fixture
def state_signal_a_fixture(mocker):
    @state(read=(DEFAULT_PROPERTY_ID,), signal_name="a")
    def state_mock_a_fn(ctx):
        pass
    state_mock_a_fn.module_name = DEFAULT_MODULE_NAME
    return state_mock_a_fn


@pytest.fixture
def state_signal_b_fixture(mocker):
    @state(signal_name="b", cond=s("module:a"))
    def state_mock_b_fn(ctx):
        pass
    state_mock_b_fn.module_name = DEFAULT_MODULE_NAME
    return state_mock_b_fn


@pytest.fixture
def state_signal_c_fixture(mocker):
    @state(signal_name="c", cond=s("module:a"))
    def state_mock_c_fn(ctx):
        pass
    state_mock_c_fn.module_name = DEFAULT_MODULE_NAME
    return state_mock_c_fn


@pytest.fixture
def state_signal_d_fixture(mocker):
    @state(signal_name="d", cond=s("module:b")|s("module:c"))
    def state_mock_c_fn(ctx):
        pass
    state_mock_c_fn.module_name = DEFAULT_MODULE_NAME
    return state_mock_c_fn


@pytest.fixture
def context_fixture(mocker):
    return Context()


@pytest.fixture
def context_with_property_fixture(mocker, context_fixture) -> Context:
    prop = PropertyBase(name=DEFAULT_PROPERTY_NAME, default_value=DEFAULT_PROPERTY_VALUE)
    prop.set_parent_path(DEFAULT_MODULE_NAME)
    context_fixture.add_prop(prop=prop)
    mocker.patch.object(context_fixture, 'add_prop')
    return context_fixture


@pytest.fixture
def context_with_property_and_state_fixture(mocker, context_with_property_fixture, state_fixture):
    context_with_property_fixture.add_state(st=state_fixture)
    mocker.patch.object(context_with_property_fixture, 'add_state')
    return context_with_property_fixture


@pytest.fixture
def context_wrapper_fixture(context_with_property_fixture, state_fixture):
    return ContextWrapper(ctx=context_with_property_fixture, st=state_fixture)


@pytest.fixture
def activation_fixture(state_fixture: State, context_with_property_and_state_fixture: Context):
    return Activation(state_fixture, context_with_property_and_state_fixture)


@pytest.fixture
def activation_fixture_fallback(activation_fixture: Activation):
    activation_fixture.state_to_activate.write_props = None
    return activation_fixture


@pytest.fixture
def spike_fixture():
    return Spike(sig=DEFAULT_PROPERTY_CHANGED)


@pytest.fixture
def triple_fixture(mocker):
    token_mock = mocker.Mock()
    from ravestate_nlp import Triple
    return Triple(token_mock, token_mock, token_mock, token_mock)
