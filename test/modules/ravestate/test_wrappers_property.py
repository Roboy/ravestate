import pytest
from ravestate.constraint import s
from testfixtures import LogCapture

from ravestate.icontext import IContext
from ravestate.property import PropertyBase
from ravestate.wrappers import PropertyWrapper
from reggol import strip_prefix


DEFAULT_MODULE_NAME = 'poo'
DEFAULT_PROPERTY_NAME = 'foo'
DEFAULT_PROPERTY_VALUE = 'Kruder'
NEW_PROPERTY_VALUE = 'Dorfmeister'


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
    prop.module_name = DEFAULT_MODULE_NAME
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
