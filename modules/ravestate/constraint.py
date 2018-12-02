from typing import List, Set

from reggol import get_logger
logger = get_logger(__name__)

def s(signalname: str):
    """
    Alias to call Signal-constructor

    :param signalname: Name of the Signal
    """
    return Signal(signalname)


class Constraint:
    """
    Superclass for Signal, Conjunct and Disjunct
    """

    def get_all_signals(self) -> Set:
        logger.error("Don't call this method on the super class Constraint")
        return set()

    def set_signal_true(self, signal):
        logger.error("Don't call this method on the super class Constraint")
        pass

    def evaluate(self) -> bool:
        logger.error("Don't call this method on the super class Constraint")
        return False


class Signal(Constraint):
    """
    Class that represents a Signal
    """
    name: str = ""
    fulfilled: bool = False

    def __init__(self, name: str):
        self.name = name

    def __or__(self, other):
        if isinstance(other, Signal):
            return Disjunct(Conjunct(self), Conjunct(other))
        elif isinstance(other, Conjunct):
            return Disjunct(Conjunct(self), other)
        elif isinstance(other, Disjunct):
            return Disjunct(self, *other)

    def __and__(self, other):
        if isinstance(other, Signal):
            return Conjunct(self, other)
        elif isinstance(other, Conjunct):
            return Conjunct(self, *other)
        elif isinstance(other, Disjunct):
            conjunct_list: List[Conjunct] = []
            for conjunct in other:
                conjunct_list.append(Conjunct(*conjunct, self))
            return Disjunct(*conjunct_list)

    def __eq__(self, other):
        return isinstance(other, Signal) and self.name == other.name

    def __hash__(self):
        return hash(self.name)

    def get_all_signals(self) -> Set:
        return {self}

    def set_signal_true(self, signal):
        if self == signal:
            self.fulfilled = True

    def evaluate(self) -> bool:
        return self.fulfilled

    def __str__(self):
        return self.name


class Conjunct(Constraint):
    """
    Class that represents a Conjunction of Signals
    """
    _signals: Set[Signal] = set()

    def __init__(self, *args):
        for arg in args:
            if not isinstance(arg, Signal):
                logger.error("Conjunct can only be constructed with Signals.")
                raise ValueError
        self._signals = set(args)

    def __iter__(self):
        for signal in self._signals:
            yield signal

    def __or__(self, other):
        if isinstance(other, Signal):
            return Disjunct(self, Conjunct(other))
        elif isinstance(other, Conjunct):
            return Disjunct(self, other)
        elif isinstance(other, Disjunct):
            return Disjunct(self, *other)

    def __and__(self, other):
        if isinstance(other, Signal):
            return Conjunct(other, *self)
        elif isinstance(other, Conjunct):
            return Conjunct(*self, *other)
        elif isinstance(other, Disjunct):
            conjunct_list: List[Conjunct] = []
            for conjunct in other:
                conjunct_list.append(Conjunct(*conjunct, *self))
            return Disjunct(*conjunct_list)

    def get_all_signals(self) -> Set[Signal]:
        return self._signals

    def set_signal_true(self, signal: Signal):
        for si in self._signals:
            si.set_signal_true(signal)

    def evaluate(self) -> bool:
        return all(map(lambda si: si.evaluate(), self._signals))

    def __str__(self):
        return "(" + " & ".join(map(lambda si: si.__str__(), self._signals)) + ")"


class Disjunct(Constraint):
    """
    Class that represents a Disjunction of Conjunctions
    """
    _conjunctions: Set[Conjunct] = set()

    def __init__(self, *args):
        for arg in args:
            if not isinstance(arg, Conjunct):
                logger.error("Disjunct can only be constructed with conjuncts.")
                raise ValueError
        self._conjunctions = set(args)

    def __iter__(self):
        for conjunct in self._conjunctions:
            yield conjunct

    def __or__(self, other):
        if isinstance(other, Signal):
            return Disjunct(*self, Conjunct(other))
        elif isinstance(other, Conjunct):
            return Disjunct(*self, other)
        elif isinstance(other, Disjunct):
            return Disjunct(*self, *other)

    def __and__(self, other):
        if isinstance(other, Signal):
            conjunct_list: List[Conjunct] = []
            for conjunct in self:
                conjunct_list.append(Conjunct(*conjunct, other))
            return Disjunct(*conjunct_list)
        elif isinstance(other, Conjunct):
            conjunct_list: List[Conjunct] = []
            for conjunct in self:
                conjunct_list.append(Conjunct(*conjunct, *other))
            return Disjunct(*conjunct_list)
        elif isinstance(other, Disjunct):
            logger.error("Can't conjunct two disjunctions.")
            raise ValueError("Can't conjunct two disjunctions.")

    def get_all_signals(self) -> Set[Signal]:
        return {signal for conjunct in self._conjunctions for signal in conjunct._signals}

    def set_signal_true(self, signal: Signal):
        for conjunct in self._conjunctions:
            conjunct.set_signal_true(signal)

    def evaluate(self) -> bool:
        return any(map(lambda si: si.evaluate(), self._conjunctions))

    def __str__(self):
        return " | ".join(map(lambda conjunct: conjunct.__str__(), self._conjunctions))
