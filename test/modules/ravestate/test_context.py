import pytest
import logging
from testfixtures import LogCapture

from ravestate.icontext import IContext
from ravestate.context import Context


@pytest.fixture
def context_mock(mocker):
    return mocker.Mock(name=IContext.__class__)


@pytest.fixture
def default_signal():
    return 'test_signal'


@pytest.fixture
def default_module():
    return 'test_module'


@pytest.fixture
def under_test():
    return Context()


def test_emit(mocker, under_test, default_signal):
    mocker.spy(under_test.signal_queue_counter, 'release')
    under_test.emit(default_signal)

    assert len(under_test.signal_queue) == 1
    assert default_signal in under_test.signal_queue
    assert under_test.signal_queue_counter.release.call_count == 1


def test_run(mocker, under_test):
    under_test.emit = mocker.stub()
    under_test.shutdown_flag = True
    under_test.run()
    under_test.emit.assert_called_once_with(':startup')


def test_run_error(under_test):
    under_test.shutdown_flag = True
    under_test.run()
    with LogCapture() as log_capture:
        under_test.run()
        log_capture.check(
            ('root', 'ERROR', 'Attempt to start context twice!'),
        )


def test_shutting_down(under_test):
    under_test.shutdown_flag = False
    assert under_test.shutting_down() is False

    under_test.shutdown_flag = True
    assert under_test.shutting_down() is True


def test_shutdown(mocker, under_test):
    under_test.shutdown_flag = False
    under_test.run_task = mocker.Mock()
    mocker.patch.object(under_test.run_task, 'join')

    under_test.emit = mocker.stub(name='emit')
    under_test.shutdown()
    under_test.emit.assert_called_once_with(':shutdown')
    under_test.run_task.join.assert_called_once()


def test_add_module_new(mocker, under_test, default_module):
    from ravestate import registry
    with mocker.patch('ravestate.registry.has_module', return_value=False):
        with mocker.patch('ravestate.registry.import_module'):
            under_test.add_module(default_module)
            registry.import_module.assert_called_once_with(
                module_name=default_module,
                callback=under_test._module_registration_callback
            )


def test_add_module_present(mocker, under_test, default_module):
    with mocker.patch('ravestate.registry.has_module', return_value=True):
        with mocker.patch.object(under_test, '_module_registration_callback'):
            under_test.add_module(default_module)
            under_test._module_registration_callback.assert_called_once()


def test_add_state():
    pass
