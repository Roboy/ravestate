import pytest
from ravestate.constraint import s


def test_signal():
    sig = s("mysig")
    assert not sig.evaluate()
    assert sig.get_all_signals() == {s("mysig")}
    sig.acquire(s("notmysig"))
    assert not sig.evaluate()
    sig.acquire(s("mysig"))
    assert sig.evaluate()

    sig_and_dis = s("sig") & (s("dis") | s("junct"))
    assert not sig_and_dis.evaluate()
    sig_and_dis.acquire(s("sig"))
    assert not sig_and_dis.evaluate()
    sig_and_dis.acquire(s("junct"))
    assert sig_and_dis.evaluate()


def test_conjunct():
    conjunct = s("sig1") & s("sig2") & s("sig3")
    assert not conjunct.evaluate()
    assert conjunct.get_all_signals() == {s("sig1"), s("sig2"), s("sig3")}
    conjunct.acquire(s("sig1"))
    assert not conjunct.evaluate()
    conjunct.acquire(s("sig2"))
    assert not conjunct.evaluate()
    conjunct.acquire(s("sig2"))
    assert not conjunct.evaluate()
    conjunct.acquire(s("sig3"))
    assert conjunct.evaluate()


def test_disjunct():
    disjunct = (s("sig1") & s("sig2")) | s("sig3")
    assert not disjunct.evaluate()
    assert disjunct.get_all_signals() == {s("sig1"), s("sig2"), s("sig3")}
    disjunct.acquire(s("sig1"))
    assert not disjunct.evaluate()
    disjunct.acquire(s("sig3"))
    assert disjunct.evaluate()


def test_legal():
    with pytest.raises(ValueError):
        v = (s("i") | s("am")) & (s("also") | s("illegal"))
