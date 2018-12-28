from typing import List, Set, Generator, Optional, Tuple
from ravestate.siginst import SignalInstance
from ravestate.iactivation import IActivation

from reggol import get_logger
logger = get_logger(__name__)


def s(signal_name: str):
    """
    Alias to call Signal-constructor

    :param signal_name: Name of the Signal
    """
    return Signal(signal_name)


class Constraint:
    """
    Superclass for Signal, Conjunct and Disjunct
    """

    def signals(self) -> Generator['Signal', None, None]:
        logger.error("Don't call this method on the super class Constraint")
        yield None

    def conjunctions(self) -> Generator['Conjunct', None, None]:
        logger.error("Don't call this method on the super class Constraint")
        yield None

    def acquire(self, signal: SignalInstance, act: IActivation):
        logger.error("Don't call this method on the super class Constraint")
        pass

    def evaluate(self) -> bool:
        logger.error("Don't call this method on the super class Constraint")
        return False

    def dereference(self, sig: Optional[SignalInstance]=None) -> Generator[Tuple['Signal', 'SignalInstance'], None, None]:
        logger.error("Don't call this method on the super class Constraint")
        yield None, None

    def update(self, act: IActivation) -> List['Signal']:
        logger.error("Don't call this method on the super class Constraint")
        return []


class Signal(Constraint):
    """
    Class that represents a Signal
    """
    name: str
    signal_instance: SignalInstance
    min_age: int
    max_age: int

    def __init__(self, name: str):
        self.name = name
        self.min_age = 0
        self.max_age = 10000
        self.signal_instance = None

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

    def signals(self) -> Generator['Signal', None, None]:
        yield self

    def conjunctions(self) -> Generator['Conjunct', None, None]:
        yield Conjunct(self)

    def acquire(self, signal: SignalInstance, act: IActivation):
        if self.name == signal.name() and self.min_age <= signal.age() <= self.max_age:
            self.signal_instance = signal
            with signal.causal_group() as cg:
                cg.acquired(signal, act)
            return True
        return False

    def evaluate(self) -> bool:
        return self.signal_instance is not None

    def dereference(self, sig: Optional[SignalInstance]=None) -> Generator[Tuple['Signal', 'SignalInstance'], None, None]:
        if (not sig and self.signal_instance) or (self.signal_instance is sig):
            former_signal_instance = self.signal_instance
            self.signal_instance = None
            yield self, former_signal_instance

    def update(self, act: IActivation) -> List['Signal']:
        if self.signal_instance and self.signal_instance.age() > self.max_age:
            with self.signal_instance.causal_group() as cg:
                cg.rejected(self.signal_instance, act)
                self.signal_instance = None
                return [self]
        return []

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

    def signals(self) -> Generator['Signal', None, None]:
        return (sig for sig in self._signals)

    def conjunctions(self) -> Generator['Conjunct', None, None]:
        yield self

    def acquire(self, signal: SignalInstance, act: IActivation):
        result = False
        for si in self._signals:
            result |= si.acquire(signal, act)
        return result

    def evaluate(self) -> bool:
        return all(map(lambda si: si.evaluate(), self._signals))

    def dereference(self, sig: Optional[SignalInstance]=None) -> Generator[Tuple['Signal', 'SignalInstance'], None, None]:
        return (result for child in self._signals for result in child.dereference(sig))

    def update(self, act: IActivation) -> List['Signal']:
        return sum((child.update(act) for child in self._signals), [])

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

    def signals(self) -> Generator['Signal', None, None]:
        return (signal for conjunct in self._conjunctions for signal in conjunct._signals)

    def conjunctions(self) -> Generator['Conjunct', None, None]:
        return (conj for conj in self._conjunctions)

    def acquire(self, signal: SignalInstance, act: IActivation):
        result = False
        for conjunct in self._conjunctions:
            result |= conjunct.acquire(signal, act)
        return result

    def evaluate(self) -> bool:
        return any(map(lambda si: si.evaluate(), self._conjunctions))

    def dereference(self, sig: Optional[SignalInstance]=None) -> Generator[Tuple['Signal', 'SignalInstance'], None, None]:
        return (result for child in self._conjunctions for result in child.dereference(sig))

    def update(self, act: IActivation) -> List['Signal']:
        return sum((child.update(act) for child in self._conjunctions), [])

    def __str__(self):
        return " | ".join(map(lambda conjunct: conjunct.__str__(), self._conjunctions))
