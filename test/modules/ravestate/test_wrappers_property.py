from ravestate.testfixtures import *
from ravestate.constraint import s
from ravestate.icontext import IContext
from ravestate.state import State
from ravestate.property import PropertyBase
from ravestate.wrappers import PropertyWrapper, ContextWrapper

DEFAULT_MODULE_NAME = 'module'
DEFAULT_PROPERTY_NAME = 'property'
DEFAULT_PROPERTY_VALUE = 'Kruder'
NEW_PROPERTY_VALUE = 'Dorfmeister'
CHILD_PROPERTY_NAME = 'child'
CHILD_PROPERTY_VALUE = 'I am a child'
GRANDCHILD_PROPERTY_NAME = 'grandchild'
GRANDCHILD_PROPERTY_VALUE = 'I am a grandchild'
CHILD_PROPERTY_FULLNAME = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}"
GRANDCHILD_PROPERTY_FULLNAME = f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}:{GRANDCHILD_PROPERTY_NAME}"


@pytest.fixture
def context_mock(mocker):
    context_mock = mocker.Mock(name=IContext.__class__)
    mocker.patch.object(context_mock, 'emit')
    mocker.patch.object(context_mock, 'add_state')
    mocker.patch.object(context_mock, 'shutdown')
    mocker.patch.object(context_mock, 'shutting_down')
    return context_mock


@pytest.fixture
def state_mock(mocker):
    state_mock = mocker.Mock(name=State.__class__)
    state_mock.module_name = DEFAULT_MODULE_NAME
    state_mock.read_props = ()
    state_mock.resources = ()
    return state_mock


@pytest.fixture
def default_property_base():
    prop = PropertyBase(name=DEFAULT_PROPERTY_NAME, default_value=DEFAULT_PROPERTY_VALUE)
    prop.set_parent_path(DEFAULT_MODULE_NAME)
    return prop


@pytest.fixture
def under_test_nothing(default_property_base, context_mock):
    return PropertyWrapper(prop=default_property_base, ctx=context_mock, allow_read=False, allow_write=False)


@pytest.fixture
def under_test_read_only(default_property_base, context_mock):
    return PropertyWrapper(prop=default_property_base, ctx=context_mock, allow_read=True, allow_write=False)


@pytest.fixture
def under_test_read_write(default_property_base, context_mock):
    return PropertyWrapper(prop=default_property_base, ctx=context_mock, allow_read=True, allow_write=True)


@pytest.fixture
def under_test_context_wrapper(context_mock, state_mock):
    return ContextWrapper(ctx=context_mock, st=state_mock)


def test_property(under_test_read_only: PropertyWrapper, default_property_base: PropertyBase):
    assert (default_property_base.id() == f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}")
    assert (not default_property_base._lock.locked())
    assert (default_property_base.read() == DEFAULT_PROPERTY_VALUE)


def test_property_get(under_test_read_only: PropertyWrapper, default_property_base: PropertyBase):
    assert (not default_property_base._lock.locked())
    assert (under_test_read_only.get() == DEFAULT_PROPERTY_VALUE)


def test_property_no_read(under_test_nothing: PropertyWrapper, default_property_base: PropertyBase):
    assert (not default_property_base._lock.locked())
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test_nothing.get()
        log_capture.check(
            f"Unauthorized read access in property-wrapper for {under_test_nothing.prop.id()}!",
        )


def test_property_read_only(under_test_read_only: PropertyWrapper, default_property_base):
    # Make sure that writing to read-only wrapper is ineffective
    assert (not default_property_base._lock.locked())
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test_read_only.set(NEW_PROPERTY_VALUE)
        log_capture.check(
            f"Unauthorized write access in property-wrapper {under_test_read_only.prop.id()}!",
        )
    assert (under_test_read_only.get() == DEFAULT_PROPERTY_VALUE)


def test_property_write(under_test_read_write: PropertyWrapper, default_property_base, context_mock):
    # Make sure that writing to writable wrapper is effective
    assert (default_property_base._lock.locked())
    under_test_read_write.set(NEW_PROPERTY_VALUE)
    assert (under_test_read_write.get() == NEW_PROPERTY_VALUE)
    context_mock.emit.assert_called_once_with(
        s(f"{under_test_read_write.prop.id()}:changed"),
        parents=None,
        wipe=True)


def test_flag_property(context_mock):
    prop_base = PropertyBase(name="flag_prop", is_flag_property=True)
    prop_base.set_parent_path(DEFAULT_MODULE_NAME)
    prop_wrapper = PropertyWrapper(prop=prop_base, ctx=context_mock, allow_read=True, allow_write=True)
    assert (prop_base._lock.locked())
<<<<<<< HEAD
    under_test_read_write.set(True)
    assert (under_test_read_write.get() == True)
    context_mock.emit.assert_called_with(
        s(f"{under_test_read_write.prop.id()}:changed"),
=======
    prop_wrapper.set(True)
    assert (prop_wrapper.get() is True)
    context_mock.emit.assert_any_call(
        s(f"{prop_wrapper.prop.fullname()}:changed"),
        parents=None,
        wipe=True)
    context_mock.emit.assert_any_call(
        s(f"{prop_wrapper.prop.fullname()}:true"),
        parents=None,
        wipe=True)

    context_mock.emit.reset_mock()
    prop_wrapper.set(False)
    assert (prop_wrapper.get() is False)
    context_mock.emit.assert_any_call(
        s(f"{prop_wrapper.prop.fullname()}:changed"),
        parents=None,
        wipe=True)
    context_mock.emit.assert_any_call(
        s(f"{prop_wrapper.prop.fullname()}:false"),
        parents=None,
        wipe=True)

    context_mock.emit.reset_mock()
    prop_wrapper.set(None)
    assert (prop_wrapper.get() is None)
    context_mock.emit.assert_called_once_with(
        s(f"{prop_wrapper.prop.fullname()}:changed"),
>>>>>>> b21a042c31198e97e854d4b01b4d5aa74679bb54
        parents=None,
        wipe=True)


def test_property_child(under_test_read_write: PropertyWrapper, default_property_base, context_mock):
    assert under_test_read_write.push(PropertyBase(name=CHILD_PROPERTY_NAME, default_value=DEFAULT_PROPERTY_VALUE))
    assert list(under_test_read_write.enum())[0] == CHILD_PROPERTY_FULLNAME
    assert under_test_read_write.prop.children[CHILD_PROPERTY_NAME].read() == DEFAULT_PROPERTY_VALUE


def test_property_illegal_push(context_mock):
    prop_no_push = PropertyBase(name=DEFAULT_PROPERTY_NAME, default_value=DEFAULT_PROPERTY_VALUE, allow_push=False)
    prop_no_push.set_parent_path(DEFAULT_MODULE_NAME)
    wrapper = PropertyWrapper(prop=prop_no_push, ctx=context_mock, allow_read=True, allow_write=True)
    with LogCapture(attributes=strip_prefix) as log_capture:
        assert not wrapper.push(child=PropertyBase(name=CHILD_PROPERTY_NAME))
        log_capture.check(
            f'Unauthorized push in property {DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}!',
        )


def test_property_illegal_pop(context_mock):
    prop_no_pop = PropertyBase(name=DEFAULT_PROPERTY_NAME, default_value=DEFAULT_PROPERTY_VALUE, allow_pop=False)
    prop_no_pop.set_parent_path(DEFAULT_MODULE_NAME)
    wrapper = PropertyWrapper(prop=prop_no_pop, ctx=context_mock, allow_read=True, allow_write=True)
    assert wrapper.push(child=PropertyBase(name=CHILD_PROPERTY_NAME))
    with LogCapture(attributes=strip_prefix) as log_capture:
        assert not wrapper.pop(CHILD_PROPERTY_NAME)
        log_capture.check(
            f'Unauthorized pop in property {DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}!',
        )
