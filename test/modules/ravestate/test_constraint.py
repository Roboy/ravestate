from ravestate.iactivation import IActivation
from ravestate.testfixtures import *
from ravestate.constraint import s, Constraint, Disjunct, Conjunct
from ravestate.spike import Spike


@pytest.fixture
def constraint_fixture():
    return Constraint()


def test_parent(constraint_fixture, spike_fixture: Spike, activation_fixture: IActivation):
    with LogCapture(attributes=strip_prefix) as log_capture:
        return_value = list(constraint_fixture.signals())
        assert return_value == [None]
        return_value = list(constraint_fixture.conjunctions())
        assert return_value == [None]
        constraint_fixture.acquire(spike_fixture, activation_fixture)
        return_value = constraint_fixture.evaluate()
        assert return_value is False
        return_value = list(constraint_fixture.dereference())
        assert return_value == [(None, None)]
        return_value = list(constraint_fixture.update(activation_fixture))
        assert return_value == [None]
        log_capture.check("Don't call this method on the super class Constraint",
                          "Don't call this method on the super class Constraint",
                          "Don't call this method on the super class Constraint",
                          "Don't call this method on the super class Constraint",
                          "Don't call this method on the super class Constraint",
                          "Don't call this method on the super class Constraint")


def test_signal(activation_fixture):
    sig = s("mysig")

    assert not sig.evaluate()
    assert set(sig.signals()) == {s("mysig")}
    sig.acquire(Spike(sig="notmysig"), activation_fixture)
    assert not sig.evaluate()
    sig.acquire(Spike(sig="mysig"), activation_fixture)
    assert sig.evaluate()

    sig_and_dis = s("sig") & (s("dis") | s("junct"))
    assert not sig_and_dis.evaluate()
    sig_and_dis.acquire(Spike(sig="sig"), activation_fixture)
    assert not sig_and_dis.evaluate()
    sig_and_dis.acquire(Spike(sig="junct"), activation_fixture)
    assert sig_and_dis.evaluate()

    expected = [(sig, sig.spike)]
    return_value = list(sig.dereference())
    assert expected == return_value

    sig.spike = Spike(sig='mysig')
    sig.spike._age = 200
    return_value = list(sig.update(activation_fixture))
    assert return_value == [sig]

    assert str(sig) == "mysig"


def test_signal_or(mocker):
    sig = s("mysig")
    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        conjunct = s("sig1") & s("sig2")
        with mocker.patch('ravestate.constraint.Conjunct.__init__', return_value=None):
            _ = sig | conjunct
            Conjunct.__init__.assert_called_once_with(sig)
        Disjunct.__init__.assert_called_once()

    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        with mocker.patch('ravestate.constraint.Disjunct.__iter__', return_value=iter([1])):
            disjunct = s("sig1") | s("sig2")
            _ = sig | disjunct
            Disjunct.__init__.assert_called_with(sig, 1)


def test_signal_and(mocker):
    sig = s("mysig")
    conjunct = s("sig1") & s("sig2")
    disjunct = s("sig1") | s("sig2")

    with mocker.patch('ravestate.constraint.Conjunct.__init__', return_value=None):
        _ = sig & sig
        Conjunct.__init__.assert_called_once_with(sig, sig)

    with mocker.patch('ravestate.constraint.Conjunct.__init__', return_value=None):
        with mocker.patch('ravestate.constraint.Conjunct.__iter__', return_value=iter([1])):
            _ = sig & conjunct
            Conjunct.__init__.assert_called_once_with(sig, 1)

    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        with mocker.patch('ravestate.constraint.Conjunct.__init__', return_value=None):
            with mocker.patch.object(disjunct, '_conjunctions', return_value={}):
                _ = sig & disjunct
            Disjunct.__init__.assert_called_with()


def test_conjunct(activation_fixture):
    conjunct = s("sig1") & s("sig2") & s("sig3")
    assert not conjunct.evaluate()
    assert set(conjunct.signals()) == {s("sig1"), s("sig2"), s("sig3")}
    conjunct.acquire(Spike(sig="sig1"), activation_fixture)
    assert not conjunct.evaluate()
    conjunct.acquire(Spike(sig="sig2"), activation_fixture)
    assert not conjunct.evaluate()
    conjunct.acquire(Spike(sig="sig2"), activation_fixture)
    assert not conjunct.evaluate()
    conjunct.acquire(Spike(sig="sig3"), activation_fixture)
    assert conjunct.evaluate()


def test_conjunct_or(mocker):
    conjunct = s("sig1") & s("sig2") & s("sig3")
    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        conjunct2 = s("sig1") & s("sig2")
        _ = conjunct | conjunct2
        Disjunct.__init__.assert_called_once_with(conjunct, conjunct2)

    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        with mocker.patch('ravestate.constraint.Disjunct.__iter__', return_value=iter([1])):
            disjunct = s("sig1") | s("sig2")
            _ = conjunct | disjunct
            Disjunct.__init__.assert_called_with(conjunct, 1)


def test_conjunct_and(mocker):
    sig = s("mysig")
    conjunct = s("sig1") & s("sig2")
    disjunct = s("sig1") | s("sig2")

    with mocker.patch('ravestate.constraint.Conjunct.__init__', return_value=None):
        _ = conjunct & sig
        Conjunct.__init__.assert_called_once_with(sig, *conjunct)

    with mocker.patch('ravestate.constraint.Conjunct.__init__', return_value=None):
        with mocker.patch('ravestate.constraint.Conjunct.__iter__', return_value=iter([1])):
            _ = conjunct & conjunct
            Conjunct.__init__.assert_called_once_with(1)

    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        with mocker.patch('ravestate.constraint.Conjunct.__init__', return_value=None):
            with mocker.patch.object(disjunct, '_conjunctions', return_value={}):
                _ = conjunct & disjunct
            Disjunct.__init__.assert_called_with()


def test_disjunct(activation_fixture):
    disjunct = (s("sig1") & s("sig2")) | s("sig3")
    assert not disjunct.evaluate()
    assert set(disjunct.signals()) == {s("sig1"), s("sig2"), s("sig3")}
    disjunct.acquire(Spike(sig="sig1"), activation_fixture)
    assert not disjunct.evaluate()
    disjunct.acquire(Spike(sig="sig3"), activation_fixture)
    assert disjunct.evaluate()


def test_disjunct_or(mocker):
    disjunct = (s("sig1") & s("sig2")) | s("sig3")
    with mocker.patch('ravestate.constraint.Disjunct.__iter__', return_value=iter([1])):
        with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
            conjunct = s("sig1") & s("sig2")
            _ = disjunct | conjunct
            Disjunct.__init__.assert_called_with(1, conjunct)

    with mocker.patch('ravestate.constraint.Disjunct.__iter__', return_value=iter([1])):
        with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
            signal = s("sig1")
            with mocker.patch('ravestate.constraint.Conjunct.__init__', return_value=None):
                _ = disjunct | signal
                Conjunct.__init__.assert_called_once_with(signal)
            Disjunct.__init__.assert_called_once()

    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        with mocker.patch('ravestate.constraint.Disjunct.__iter__', return_value=iter([1])):
            disjunct2 = s("sig1") | s("sig2")
            _ = disjunct | disjunct2
            Disjunct.__init__.assert_called_with(1)


def test_disjunct_and(mocker):
    sig = s("mysig")
    conjunct = s("sig1") & s("sig2")
    disjunct = s("sig1") | s("sig2")

    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        with mocker.patch.object(disjunct, '_conjunctions', return_value={}):
            _ = disjunct & sig
            Disjunct.__init__.assert_called_once_with()

    with mocker.patch('ravestate.constraint.Disjunct.__init__', return_value=None):
        with mocker.patch.object(disjunct, '_conjunctions', return_value={}):
            _ = disjunct & conjunct
            Disjunct.__init__.assert_called_once_with()

    with pytest.raises(ValueError):
        with LogCapture(attributes=strip_prefix) as log_capture:
            _ = disjunct & disjunct
            log_capture.check("Can't conjunct two disjunctions.")


def test_legal():
    with pytest.raises(ValueError):
        _ = (s("i") | s("am")) & (s("also") | s("illegal"))
