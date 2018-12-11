import pytest
from ravestate.constraint import s
from testfixtures import LogCapture

from ravestate.icontext import IContext
from ravestate.property import PropertyBase
from ravestate.wrappers import PropertyWrapper
from reggol import strip_prefix


DEFAULT_MODULE_NAME = 'module'
DEFAULT_PROPERTY_NAME = 'property'
DEFAULT_PROPERTY_VALUE = 'Kruder'
NEW_PROPERTY_VALUE = 'Dorfmeister'
CHILD_PROPERTY_NAME = 'child'
CHILD_PROPERTY_VALUE = 'I am a child'
GRANDCHILD_PROPERTY_NAME = 'grandchild'
GRANDCHILD_PROPERTY_VALUE = 'I am a grandchild'


@pytest.fixture
def context_mock(mocker):
    context_mock = mocker.Mock(name=IContext.__class__)
    mocker.patch.object(context_mock, 'emit')
    mocker.patch.object(context_mock, 'add_state')
    mocker.patch.object(context_mock, 'shutdown')
    mocker.patch.object(context_mock, 'shutting_down')
    return context_mock


@pytest.fixture
def default_property_base():
    prop = PropertyBase(name=DEFAULT_PROPERTY_NAME, default=DEFAULT_PROPERTY_VALUE)
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


def test_property(under_test_read_only: PropertyWrapper, default_property_base: PropertyBase):
    assert (default_property_base.fullname() == f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}")
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
            f"Unauthorized read access in property-wrapper for {under_test_nothing.prop.name}!",
        )


def test_property_read_only(under_test_read_only: PropertyWrapper, default_property_base):
    # Make sure that writing to read-only wrapper is ineffective
    assert (not default_property_base._lock.locked())
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test_read_only.set(NEW_PROPERTY_VALUE)
        log_capture.check(
            f"Unauthorized write access in property-wrapper {under_test_read_only.prop.name}!",
        )
    assert (under_test_read_only.get() == DEFAULT_PROPERTY_VALUE)


def test_property_write(under_test_read_write: PropertyWrapper, default_property_base, context_mock):
    # Make sure that writing to writable wrapper is effective
    assert (default_property_base._lock.locked())
    under_test_read_write.set(NEW_PROPERTY_VALUE)
    assert (under_test_read_write.get() == NEW_PROPERTY_VALUE)
    context_mock.emit.assert_called_once_with(s(f"{under_test_read_write.prop.fullname()}:changed"))


def test_property_child(under_test_read_write: PropertyWrapper, default_property_base, context_mock):
    assert under_test_read_write.push([CHILD_PROPERTY_NAME], default=DEFAULT_PROPERTY_VALUE)
    assert under_test_read_write.get(child=[CHILD_PROPERTY_NAME]) == DEFAULT_PROPERTY_VALUE
    assert under_test_read_write.set(CHILD_PROPERTY_VALUE, child=[CHILD_PROPERTY_NAME])
    assert under_test_read_write.get(child=[CHILD_PROPERTY_NAME]) == CHILD_PROPERTY_VALUE
    context_mock.emit.assert_called_once_with(
        s(f"{under_test_read_write.prop.fullname()}:{CHILD_PROPERTY_NAME}:changed"))


def test_property_nested_child(under_test_read_write: PropertyWrapper, default_property_base, context_mock):
    # test adding child and grandchild in one step
    assert under_test_read_write.push([CHILD_PROPERTY_NAME, GRANDCHILD_PROPERTY_NAME])
    assert_child(context_mock, under_test_read_write)
    assert_grandchild(context_mock, under_test_read_write)
    # test popping of child
    assert under_test_read_write.pop(child=[CHILD_PROPERTY_NAME])
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test_read_write.get([CHILD_PROPERTY_NAME])
        log_capture.check(
            f'Tried to read non-existent child-property {under_test_read_write.prop.fullname()}:{CHILD_PROPERTY_NAME}',
        )
    # test adding child and grandchild in two steps
    assert under_test_read_write.push([CHILD_PROPERTY_NAME])
    assert_child(context_mock, under_test_read_write)
    assert under_test_read_write.push([CHILD_PROPERTY_NAME, GRANDCHILD_PROPERTY_NAME])
    assert_grandchild(context_mock, under_test_read_write)
    # test popping of grandchild
    assert under_test_read_write.pop(child=[CHILD_PROPERTY_NAME, GRANDCHILD_PROPERTY_NAME])
    assert_child(context_mock, under_test_read_write)
    with LogCapture(attributes=strip_prefix) as log_capture:
        under_test_read_write.get([CHILD_PROPERTY_NAME, GRANDCHILD_PROPERTY_NAME])
        log_capture.check(
            f'Tried to read non-existent child-property {DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}:{GRANDCHILD_PROPERTY_NAME}',
        )


def assert_child(context_mock, under_test_read_write):
    under_test_read_write.set(CHILD_PROPERTY_VALUE + "old", child=[CHILD_PROPERTY_NAME])
    assert under_test_read_write.set(CHILD_PROPERTY_VALUE, child=[CHILD_PROPERTY_NAME])
    assert under_test_read_write.get(child=[CHILD_PROPERTY_NAME]) == CHILD_PROPERTY_VALUE
    context_mock.emit.assert_called_with(
        s(f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}:changed"))


def assert_grandchild(context_mock, under_test_read_write):
    under_test_read_write.set(GRANDCHILD_PROPERTY_VALUE + "old", child=[CHILD_PROPERTY_NAME, GRANDCHILD_PROPERTY_NAME])
    assert under_test_read_write.set(GRANDCHILD_PROPERTY_VALUE, child=[CHILD_PROPERTY_NAME, GRANDCHILD_PROPERTY_NAME])
    assert under_test_read_write.get(child=[CHILD_PROPERTY_NAME, GRANDCHILD_PROPERTY_NAME]) == GRANDCHILD_PROPERTY_VALUE
    context_mock.emit.assert_called_with(
        s(f"{DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}:{CHILD_PROPERTY_NAME}:{GRANDCHILD_PROPERTY_NAME}:changed"))


def test_property_illegal_push(context_mock):
    prop_no_push = PropertyBase(name=DEFAULT_PROPERTY_NAME, default=DEFAULT_PROPERTY_VALUE, allow_push=False)
    prop_no_push.set_parent_path(DEFAULT_MODULE_NAME)
    wrapper = PropertyWrapper(prop=prop_no_push, ctx=context_mock, allow_read=True, allow_write=True)
    with LogCapture(attributes=strip_prefix) as log_capture:
        assert not wrapper.push([CHILD_PROPERTY_NAME])
        log_capture.check(
            f'Unauthorized push in property {DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}!',
        )


def test_property_illegal_pop(context_mock):
    prop_no_pop = PropertyBase(name=DEFAULT_PROPERTY_NAME, default=DEFAULT_PROPERTY_VALUE, allow_pop=False)
    prop_no_pop.set_parent_path(DEFAULT_MODULE_NAME)
    wrapper = PropertyWrapper(prop=prop_no_pop, ctx=context_mock, allow_read=True, allow_write=True)
    assert wrapper.push([CHILD_PROPERTY_NAME])
    with LogCapture(attributes=strip_prefix) as log_capture:
        assert not wrapper.pop([CHILD_PROPERTY_NAME])
        log_capture.check(
            f'Unauthorized pop in property {DEFAULT_MODULE_NAME}:{DEFAULT_PROPERTY_NAME}!',
        )
