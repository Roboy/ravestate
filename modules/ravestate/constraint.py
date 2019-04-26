from typing import List, Set, Generator, Optional, Tuple, Union, Callable, Any
from ravestate.spike import Spike
from ravestate.iactivation import IActivation, ICausalGroup

from reggol import get_logger
logger = get_logger(__name__)


class ConfigurableAge:
    """
    Class for having min/max_age parameters for Constraints configurable with a config key
    """
    key = ""

    def __init__(self, key: str):
        self.key = key


def s(signal_name: str, *, min_age: Union[float, ConfigurableAge] = 0., max_age: Union[float, ConfigurableAge] = 5.,
      detached: bool = False) -> 'Signal':
    """
    Alias to call Signal-constructor

    * `signal_name`: Name of the Signal

    * `min_age`: Minimum age for the signal, in seconds. Can also be ConfigurableAge that gets the age from the config.

    * `max_age`: Maximum age for the signal, in seconds. Can also be ConfigurableAge that gets the age from the config.
     Set to less-than zero for unrestricted age.

    * `detached`: Flag which indicates, whether spikes that fulfill this signal
     are going to have a separate causal group from spikes that
     are generated by a state that uses this signal as a constraint.
    """
    return Signal(signal_name, min_age=min_age, max_age=max_age, detached=detached)


class Constraint:
    """
    Superclass for Signal, Conjunct and Disjunct
    """

    def signals(self) -> Generator['Signal', None, None]:
        logger.error("Don't call this method on the super class Constraint")
        yield None

    def conjunctions(self, filter_detached=False) -> Generator['Conjunct', None, None]:
        logger.error("Don't call this method on the super class Constraint")
        yield None

    def acquire(self, spike: Spike, act: IActivation):
        logger.error("Don't call this method on the super class Constraint")
        pass

    def evaluate(self) -> bool:
        logger.error("Don't call this method on the super class Constraint")
        return False

    def dereference(self, *,
                    spike: Optional[Spike]=None,
                    causal_groups: Optional[Set[ICausalGroup]]=None) -> Generator[Tuple['Signal', 'Spike'], None, None]:
        logger.error("Don't call this method on the super class Constraint")
        yield None, None

    def update(self, act: IActivation) -> Generator['Signal', None, None]:
        logger.error("Don't call this method on the super class Constraint")
        yield None

    def fulfilled_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        logger.error("Don't call this method on the super class Constraint")
        pass


class Signal(Constraint):
    """
    Class that represents a Signal
    """
    name: str
    spike: Spike
    min_age: float
    max_age: float
    detached: bool

    # tells whether this signal has been completed, and may therefore
    #  not introduce a new causal group into the parent conjunct
    completed_by: Optional[Set['Signal']]

    # tells whether this signal is a potential cause for another signal,
    #  and should therefore not be affected by the activation's auto-elimination
    #  death clock.
    is_completion: bool

    _min_age_ticks: int  # written on acquire, when act.secs_to_ticks is available

    def __init__(self, name: str, *, min_age=0., max_age=5., detached=False):
        self.name = name
        self.min_age = min_age
        self.max_age = max_age
        self.spike = None
        self.detached = detached
        self.completed_by = None
        self.is_completion = False
        self._min_age_ticks = 0
        # TODO: Deal with ConfigurableAge
        # if min_age > max_age and max_age > .0:
        #     logger.warning(f"{self}: max_age={max_age} < min_age={min_age}!")

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

    def __repr__(self):
        return f"Signal({self.name}, {self.min_age}, {self.max_age}, {self.detached})"

    def signals(self) -> Generator['Signal', None, None]:
        yield self

    def conjunctions(self, filter_detached=False) -> Generator['Conjunct', None, None]:
        if not filter_detached or not self.detached:
            yield Conjunct(self)

    def acquire(self, spike: Spike, act: IActivation):
        if not self.spike and self.name == spike.name() and (self.max_age < 0 or spike.age() <= act.secs_to_ticks(self.max_age)):
            assert not spike.is_wiped()
            with spike.causal_group() as cg:
                # Causal group might refuse acquisition, if one of act's state's write-props is unavailable.
                if not cg.acquired(spike, act, self.detached):
                    return False
            self._min_age_ticks = act.secs_to_ticks(self.min_age)
            self.spike = spike
            return True
        return False

    def evaluate(self) -> bool:
        return self.spike and self._min_age_ticks <= self.spike.age()

    def dereference(self, *,
                    spike: Optional[Spike]=None,
                    causal_groups: Optional[Set[ICausalGroup]]=None) -> Generator[Tuple['Signal', 'Spike'], None, None]:
        if (not spike and self.spike) or (spike and self.spike is spike):
            if causal_groups:
                # TODO: Only deref, if is_fulfilled_causal_tail(). Then also deref completed_by signals.
                with self.spike.causal_group() as cg:
                    if cg not in causal_groups:
                        return
            former_signal_instance = self.spike
            self.spike = None
            yield self, former_signal_instance

    def update(self, act: IActivation) -> Generator['Signal', None, None]:
        # Reject spike, once it has become too old
        if self.spike and self.max_age >= 0 and self.spike.age() > act.secs_to_ticks(self.max_age):
            with self.spike.causal_group() as cg:
                cg.rejected(self.spike, act, reason=1)
                self.spike = None
                yield self

    def fulfilled_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        if self.is_fulfilled_causal_tail():
            yield self.spike.causal_group()

    def is_fulfilled_causal_tail(self):
        return not self.is_completion and self.evaluate()

    def __str__(self):
        return self.name


class Conjunct(Constraint):
    """
    Class that represents a Conjunction of Signals.
    Can be constructed using an overloaded & operator.

    _Example:_
    ```python
    signal_A & signal_B
    ```
    """
    _signals: Set[Signal]
    _hash: Tuple[str]
    _allowed_causal_groups: Set[ICausalGroup]

    def __init__(self, *args):
        for arg in args:
            if not isinstance(arg, Signal):
                logger.error("Conjunct can only be constructed with Signals.")
                raise ValueError
        self._signals = set(args)
        self._hash = hash(tuple(sorted(sig.name for sig in self._signals)))
        self._allowed_causal_groups = set()

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

    def __contains__(self, item):
        return item in self._signals

    def __hash__(self):
        return self._hash

    def __eq__(self, other):
        return isinstance(other, Conjunct) and self._hash == other._hash

    def signals(self) -> Generator['Signal', None, None]:
        return (sig for sig in self._signals)

    def conjunctions(self, filter_detached=False) -> Generator['Conjunct', None, None]:
        result = self
        if filter_detached:
            result = Conjunct(*(sig for sig in self._signals if not sig.detached))
        if result._signals:
            yield result

    def acquire(self, spike: Spike, act: IActivation):
        result = False
        # update causal group set to account for merges
        self._allowed_causal_groups = set(cg for cg in self._allowed_causal_groups)
        for si in self._signals:
            if spike.causal_group() in self._allowed_causal_groups:
                # the causal group is already allowed
                result |= si.acquire(spike, act)
            elif not si.completed_by:
                if si.acquire(spike, act):
                    # add causal group to allowed, since the spike is a root cause
                    self._allowed_causal_groups.add(spike.causal_group())
                    result = True
        return result

    def evaluate(self) -> bool:
        return all(si.evaluate() for si in self._signals)

    def dereference(self, *,
                    spike: Optional[Spike]=None,
                    causal_groups: Optional[Set[ICausalGroup]]=None) -> Generator[Tuple['Signal', 'Spike'], None, None]:
        for child in self._signals:
            for result in child.dereference(spike=spike, causal_groups=causal_groups):
                self._allowed_causal_groups = {sig.spike.causal_group() for sig in self._signals if sig.spike}
                yield result

    def update(self, act: IActivation) -> Generator['Signal', None, None]:
        for child in self._signals:
            for result in child.update(act):  # If anything is returned, it means the signal rejected it's spike.
                self._allowed_causal_groups = {sig.spike.causal_group() for sig in self._signals if sig.spike}
                yield result

    def fulfilled_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        for child in self._signals:
            yield from child.fulfilled_causal_groups()

    def __str__(self):
        return "(" + " & ".join(map(lambda si: si.__str__(), self._signals)) + ")"


class Disjunct(Constraint):
    """
    Class that represents a Disjunction of Conjunctions
    Can be constructed using an overloaded | operator.

    _Examples:_
    ```python
    conjunction_A | conjunction_B

    (signal_A & signal_B) | (signal_C & signal_D)
    ```
    """
    _conjunctions: Set[Conjunct]

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

    def conjunctions(self, filter_detached=False) -> Generator['Conjunct', None, None]:
        for conj in self._conjunctions:
            yield from conj.conjunctions(filter_detached=filter_detached)

    def acquire(self, spike: Spike, act: IActivation):
        result = False
        for conjunct in self._conjunctions:
            result |= conjunct.acquire(spike, act)
        return result

    def evaluate(self) -> bool:
        return any(child.evaluate() for child in self._conjunctions)

    def dereference(self, *,
                    spike: Optional[Spike]=None,
                    causal_groups: Optional[Set[ICausalGroup]]=None) -> Generator[Tuple['Signal', 'Spike'], None, None]:
        for child in self._conjunctions:
            yield from child.dereference(spike=spike, causal_groups=causal_groups)

    def update(self, act: IActivation) -> Generator['Signal', None, None]:
        for child in self._conjunctions:
            yield from child.update(act)

    def fulfilled_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        for child in self._conjunctions:
            yield from child.fulfilled_causal_groups()

    def __str__(self):
        return " | ".join(map(lambda conjunct: conjunct.__str__(), self._conjunctions))
