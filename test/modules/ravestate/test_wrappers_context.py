import pytest
from testfixtures import LogCapture

from ravestate.context import Context
from ravestate.icontext import IContext
from ravestate.property import PropertyBase
from ravestate.state import State
from ravestate.wrappers import PropertyWrapper, ContextWrapper


DEFAULT_MODULE_NAME = 'poo'


@pytest.fixture
def state_mock(mocker):
    state = mocker.Mock(name=State.__class__)
    state.module_name = DEFAULT_MODULE_NAME
    state.read_props = ()
    state.write_props = ()
    return state


@pytest.fixture
def context_mock(mocker):
    context_mock = mocker.Mock(name=IContext.__class__)
    mocker.patch.object(context_mock, 'emit')
    mocker.patch.object(context_mock, 'add_state')
    mocker.patch.object(context_mock, 'shutdown')
    mocker.patch.object(context_mock, 'shutting_down')
    return context_mock


@pytest.fixture
def under_test(state_mock: State, context_mock: Context):
    return ContextWrapper(ctx=context_mock, st=state_mock)


def test_add_state(mocker, under_test: PropertyWrapper, context_mock: Context, state_mock: State):
    with mocker.patch('ravestate.registry.get_module', return_value=DEFAULT_MODULE_NAME):
        under_test.add_state(state_mock)
        context_mock.add_state.assert_called_once_with(st=state_mock)


def test_context_shutdown(under_test: PropertyWrapper, context_mock: Context):
    under_test.shutdown()
    context_mock.shutdown.assert_called_once()


def test_context_shutting_down(under_test: PropertyWrapper, context_mock: Context):
    under_test.shutting_down()
    context_mock.shutting_down.assert_called_once()
