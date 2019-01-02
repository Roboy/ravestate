import pytest

from ravestate.constraint import s
from ravestate.context import Context
from ravestate.property import PropertyBase
from ravestate.state import State, state
from ravestate.wrappers import PropertyWrapper, ContextWrapper
from ravestate.activation import Activation

DEFAULT_MODULE_NAME = 'module'
DEFAULT_PROPERTY_NAME = 'property'
DEFAULT_PROPERTY_FULLNAME = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}"
DEFAULT_PROPERTY_VALUE = 'Kruder'
DEFAULT_PROPERTY_CHANGED = f"{DEFAULT_PROPERTY_FULLNAME}:changed"
NEW_PROPERTY_VALUE = 'Dorfmeister'


@pytest.fixture
def state_fixture(mocker):
    @state(write=(DEFAULT_PROPERTY_FULLNAME,), read=(DEFAULT_PROPERTY_FULLNAME,))
    def state_mock_fn(ctx):
        pass
    state_mock_fn.module_name = DEFAULT_MODULE_NAME
    return state_mock_fn


@pytest.fixture
def context_fixture(mocker):
    return Context()


@pytest.fixture
def context_with_property_fixture(mocker, context_fixture):
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
    return ContextWrapper(context_with_property_fixture, state_fixture)


@pytest.fixture
def activation_fixture(state_fixture: State, context_with_property_and_state_fixture: Context):
    return Activation(state_fixture, context_with_property_and_state_fixture)
