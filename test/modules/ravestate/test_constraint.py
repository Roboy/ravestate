from ravestate.testfixtures import *
from ravestate.constraint import s
from ravestate.spike import Spike


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


def test_disjunct(activation_fixture):
    disjunct = (s("sig1") & s("sig2")) | s("sig3")
    assert not disjunct.evaluate()
    assert set(disjunct.signals()) == {s("sig1"), s("sig2"), s("sig3")}
    disjunct.acquire(Spike(sig="sig1"), activation_fixture)
    assert not disjunct.evaluate()
    disjunct.acquire(Spike(sig="sig3"), activation_fixture)
    assert disjunct.evaluate()


def test_legal():
    with pytest.raises(ValueError):
        v = (s("i") | s("am")) & (s("also") | s("illegal"))
