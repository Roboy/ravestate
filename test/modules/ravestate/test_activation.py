import pytest
from threading import Thread

from ravestate.activation import StateActivation
from ravestate.constraint import s
from ravestate.state import State
from ravestate.icontext import IContext


@pytest.fixture
def state_mock(mocker):
    state = mocker.Mock(name=State.__class__)
    state.triggers = (s('test1') & s('test2')) | s('test3')
    return state


@pytest.fixture
def context_mock(mocker):
    return mocker.Mock(name=IContext.__class__)


@pytest.fixture
def under_test(state_mock: State, context_mock: IContext):
    return StateActivation(state_mock, context_mock)


def test_specifity(under_test):
    assert under_test.specificity() == 1


def test_notify_signal(under_test):
    assert under_test.acquire(s('test1')) == 0
    assert under_test.acquire(s('test2')) == 1


def test_notify_signal_2(under_test):
    assert under_test.acquire(s('test1')) == 0
    assert under_test.acquire(s('test3')) == 1


def test_notify_signal_mismatched(under_test):
    assert under_test.acquire('') == 0


def test_notify_signal_mismatched_2(under_test):
    assert under_test.acquire(s('test1')) == 0
    assert under_test.acquire(s('notest')) == 0


def test_multiple_activation(state_mock, context_mock):
    sa1 = StateActivation(state_mock, context_mock)
    assert sa1.acquire(s('test1')) == 0
    assert sa1.acquire(s('test3')) == 1
    sa2 = StateActivation(state_mock, context_mock)
    assert sa2.acquire(s('test1')) == 0
    assert sa2.acquire(s('test3')) == 1


# TODO: Add tests for private run
def test_run(under_test):
    result = under_test.run()
    assert isinstance(result, Thread)
