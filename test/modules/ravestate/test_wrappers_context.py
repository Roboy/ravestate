import pytest

from ravestate.constraint import s
from ravestate.context import Context
from ravestate.icontext import IContext
from ravestate.property import PropertyBase
from ravestate.state import State
from ravestate.wrappers import PropertyWrapper, ContextWrapper

DEFAULT_MODULE_NAME = 'module'
DEFAULT_PROPERTY_NAME = 'property'
DEFAULT_PROPERTY_VALUE = 'Kruder'
NEW_PROPERTY_VALUE = 'Dorfmeister'
CHILD_PROPERTY_NAME = 'child'
CHILD_PROPERTY_VALUE = 'I am a child'
GRANDCHILD_PROPERTY_NAME = 'grandchild'
GRANDCHILD_PROPERTY_VALUE = 'I am a grandchild'
DEFAULT_PROPERTY_FULLNAME = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}"
CHILD_PROPERTY_FULLNAME = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}"
GRANDCHILD_PROPERTY_FULLNAME = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}:{GRANDCHILD_PROPERTY_NAME}"


@pytest.fixture
def state_mock(mocker):
    state = mocker.Mock(name=State.__class__)
    state.module_name = DEFAULT_MODULE_NAME
    state.read_props = (DEFAULT_PROPERTY_FULLNAME,)
    state.write_props = (DEFAULT_PROPERTY_FULLNAME,)
    state.name = "mockstate"
    return state


@pytest.fixture
def context_mock(mocker):
    context_mock = mocker.Mock(name=IContext.__class__)
    mocker.patch.object(context_mock, 'emit')
    mocker.patch.object(context_mock, 'add_state')
    mocker.patch.object(context_mock, 'shutdown')
    mocker.patch.object(context_mock, 'shutting_down')
    mocker.patch.object(context_mock, 'get_prop')
    return context_mock


@pytest.fixture
def under_test(state_mock: State, context_mock: Context):
    return ContextWrapper(ctx=context_mock, st=state_mock)


@pytest.fixture
def push_pop_context(mocker, state_mock):
    context = Context()
    prop = PropertyBase(name=DEFAULT_PROPERTY_NAME, default=DEFAULT_PROPERTY_VALUE)
    prop.set_parent_path(DEFAULT_MODULE_NAME)
    context.add_prop(prop=prop)
    mocker.patch.object(context, 'emit')
    return context


@pytest.fixture
def push_pop_wrapper(push_pop_context, state_mock):
    return ContextWrapper(push_pop_context, state_mock)


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


def test_property_push_pop(push_pop_wrapper: ContextWrapper, push_pop_context):
    # push child
    assert push_pop_wrapper.push(parentpath=DEFAULT_PROPERTY_FULLNAME,
                                 child=PropertyBase(name=CHILD_PROPERTY_NAME, default=DEFAULT_PROPERTY_VALUE))
    push_pop_context.emit.assert_called_with(s(f"{DEFAULT_PROPERTY_FULLNAME}:pushed"))

    # test child
    assert CHILD_PROPERTY_FULLNAME in list(push_pop_wrapper.enum(DEFAULT_PROPERTY_FULLNAME))
    assert push_pop_wrapper[CHILD_PROPERTY_FULLNAME] == DEFAULT_PROPERTY_VALUE
    push_pop_wrapper[CHILD_PROPERTY_FULLNAME] = CHILD_PROPERTY_VALUE
    push_pop_context.emit.assert_called_with(s(f"{CHILD_PROPERTY_FULLNAME}:changed"))
    assert push_pop_wrapper[CHILD_PROPERTY_FULLNAME] == CHILD_PROPERTY_VALUE

    # pop child
    assert push_pop_wrapper.pop(CHILD_PROPERTY_FULLNAME)
    push_pop_context.emit.assert_called_with(s(f"{DEFAULT_PROPERTY_FULLNAME}:popped"))
    assert [] == list(push_pop_wrapper.enum(DEFAULT_PROPERTY_FULLNAME))


def test_property_nested(push_pop_wrapper: ContextWrapper, push_pop_context):
    # push child
    assert push_pop_wrapper.push(parentpath=DEFAULT_PROPERTY_FULLNAME,
                                 child=PropertyBase(name=CHILD_PROPERTY_NAME))
    push_pop_context.emit.assert_called_with(s(f"{DEFAULT_PROPERTY_FULLNAME}:pushed"))

    # push grandchild
    assert push_pop_wrapper.push(parentpath=CHILD_PROPERTY_FULLNAME,
                                 child=PropertyBase(name=GRANDCHILD_PROPERTY_NAME, default=DEFAULT_PROPERTY_VALUE))
    push_pop_context.emit.assert_called_with(s(f"{CHILD_PROPERTY_FULLNAME}:pushed"))

    # test children
    assert CHILD_PROPERTY_FULLNAME in list(push_pop_wrapper.enum(DEFAULT_PROPERTY_FULLNAME))
    assert GRANDCHILD_PROPERTY_FULLNAME in list(push_pop_wrapper.enum(CHILD_PROPERTY_FULLNAME))
    assert push_pop_wrapper[GRANDCHILD_PROPERTY_FULLNAME] == DEFAULT_PROPERTY_VALUE
    push_pop_wrapper[GRANDCHILD_PROPERTY_FULLNAME] = GRANDCHILD_PROPERTY_VALUE
    push_pop_context.emit.assert_called_with(s(f"{GRANDCHILD_PROPERTY_FULLNAME}:changed"))
    assert push_pop_wrapper[GRANDCHILD_PROPERTY_FULLNAME] == GRANDCHILD_PROPERTY_VALUE

    # pop child
    assert push_pop_wrapper.pop(CHILD_PROPERTY_FULLNAME)
    push_pop_context.emit.assert_called_with(s(f"{DEFAULT_PROPERTY_FULLNAME}:popped"))
    assert [] == list(push_pop_wrapper.enum(DEFAULT_PROPERTY_FULLNAME))


def test_property_nested_2(push_pop_wrapper: ContextWrapper, push_pop_context):
    # push child
    assert push_pop_wrapper.push(parentpath=DEFAULT_PROPERTY_FULLNAME,
                                 child=PropertyBase(name=CHILD_PROPERTY_NAME))
    push_pop_context.emit.assert_called_with(s(f"{DEFAULT_PROPERTY_FULLNAME}:pushed"))

    # push grandchild
    assert push_pop_wrapper.push(parentpath=CHILD_PROPERTY_FULLNAME,
                                 child=PropertyBase(name=GRANDCHILD_PROPERTY_NAME))
    push_pop_context.emit.assert_called_with(s(f"{CHILD_PROPERTY_FULLNAME}:pushed"))

    # test children
    assert CHILD_PROPERTY_FULLNAME in list(push_pop_wrapper.enum(DEFAULT_PROPERTY_FULLNAME))
    assert GRANDCHILD_PROPERTY_FULLNAME in list(push_pop_wrapper.enum(CHILD_PROPERTY_FULLNAME))

    # pop grandchild
    assert push_pop_wrapper.pop(GRANDCHILD_PROPERTY_FULLNAME)
    push_pop_context.emit.assert_called_with(s(f"{CHILD_PROPERTY_FULLNAME}:popped"))
    assert [] == list(push_pop_wrapper.enum(CHILD_PROPERTY_FULLNAME))
    assert CHILD_PROPERTY_FULLNAME in list(push_pop_wrapper.enum(DEFAULT_PROPERTY_FULLNAME))
