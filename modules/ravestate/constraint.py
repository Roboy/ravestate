import copy
from typing import List, Set, Generator, Optional, Tuple, Union, Callable, Any
from ravestate.spike import Spike
from ravestate.iactivation import IActivation, ICausalGroup
from ravestate.threadlocal import ravestate_thread_local


from reggol import get_logger
logger = get_logger(__name__)


class ConfigurableAge:
    """
    Class for having min/max_age parameters for Constraints configurable with a config key
    """
    key = ""

    def __init__(self, key: str):
        self.key = key


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

    def referenced_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        logger.error("Don't call this method on the super class Constraint")
        pass

    def fulfilled_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        logger.error("Don't call this method on the super class Constraint")
        pass

    def effect_not_caused(self, act: IActivation, group: ICausalGroup, effect: str) -> Generator['Signal', None, None]:
        logger.error("Don't call this method on the super class Constraint")
        pass


class Signal(Constraint):
    """
    Class that represents a Signal. Should be constructed in a `with Module(..)`
     context, such that it's module scope is set automatically.
    """

    name: str
    spike: Optional[Spike]
    min_age_value: float
    max_age_value: float
    detached_value: bool
    parent_path: str

    # tells whether this signal has been completed, and may therefore
    #  not introduce a new causal group into the parent conjunct.
    #  the back-references to the completing signals allow it
    #  to dereference the completing signals when the causal tail
    #  is dereferences because of death_clock.
    completed_by: Optional[Set['Signal']]

    # tells whether this signal is a potential cause for another signal,
    #  and should therefore not be (immediately) affected by the activation's
    #  auto-elimination death clock.
    is_completion: bool

    _min_age_ticks: int  # written on acquire, when act.secs_to_ticks is available

    def __init__(self, name: str, *, min_age=0., max_age=5., detached=False, _skip_module_context=False):
        self.name = name
        self.min_age_value = min_age
        self.max_age_value = max_age
        self.spike = None
        self.detached_value = detached
        self.completed_by = None
        self.is_completion = False
        self._min_age_ticks = 0
        self.parent_path = ""
        # TODO: Deal with ConfigurableAge
        # if min_age > max_age and max_age > .0:
        #     logger.warning(f"{self}: max_age={max_age} < min_age={min_age}!")

        # add signal to module in current `with Module(...)` clause
        module_under_construction = getattr(ravestate_thread_local, 'module_under_construction', None)
        if module_under_construction:
            module_under_construction.add(self)

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
        return (isinstance(other, Signal) and self.id() == other.id()) or \
               (isinstance(other, str) and self.id() == other)

    def __hash__(self):
        return hash(self.id())

    def __repr__(self):
        return f"Signal({self.id()}, {self.min_age_value}, {self.max_age_value}, {self.detached_value})"

    def __str__(self):
        return self.id()

    def id(self):
        return f'{self.parent_path}:{self.name}'

    def signals(self) -> Generator['Signal', None, None]:
        yield self

    def conjunctions(self, filter_detached=False) -> Generator['Conjunct', None, None]:
        if not filter_detached or not self.detached_value:
            yield Conjunct(self)

    def acquire(self, spike: Spike, act: IActivation):
        if not self.spike and self.id() == spike.id() and (self.max_age_value < 0 or spike.age() <= act.secs_to_ticks(self.max_age_value)):
            assert not spike.is_wiped()
            with spike.causal_group() as cg:
                # Causal group might refuse acquisition, if one of act's state's write-props is unavailable.
                if not cg.acquired(spike, act, self.detached_value):
                    return False
                self.spike = spike
            self._min_age_ticks = act.secs_to_ticks(self.min_age_value)
            return True
        return False

    def evaluate(self) -> bool:
        return self.spike and self._min_age_ticks <= self.spike.age()

    def dereference(self, *,
                    spike: Optional[Spike]=None,
                    causal_groups: Optional[Set[ICausalGroup]]=None) -> Generator[Tuple['Signal', 'Spike'], None, None]:
        if (not spike and self.spike) or (spike and self.spike is spike):
            if causal_groups:
                if self.is_fulfilled_causal_tail():
                    with self.spike.causal_group() as cg:
                        if cg not in causal_groups:
                            return
                    # Also dereference other spikes from this causal chain
                    if self.completed_by:
                        for completing_signal in self.completed_by:
                            yield from completing_signal.dereference()
                else:
                    return
            former_signal_instance = self.spike
            self.spike = None
            yield self, former_signal_instance

    def update(self, act: IActivation) -> Generator['Signal', None, None]:
        # Reject spike, once it has become too old
        if self.spike and self.max_age_value >= 0 and self.spike.age() > act.secs_to_ticks(self.max_age_value):
            with self.spike.causal_group() as cg:
                cg.rejected(self.spike, act, reason=1)
                self.spike = None
                yield self

    def referenced_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        if self.spike:
            yield self.spike.causal_group()

    def fulfilled_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        if self.is_fulfilled_causal_tail():
            yield self.spike.causal_group()

    def is_fulfilled_causal_tail(self):
        return not self.is_completion and self.evaluate()

    def effect_not_caused(self, act: IActivation, group: ICausalGroup, effect: str) -> Generator['Signal', None, None]:
        if self.completed_by:
            for cause in self.completed_by:
                if cause.spike:
                    with cause.spike.causal_group() as cg:
                        if cg == group:
                            cg.rejected(cause.spike, act, reason=1)
                            cause.spike = None
                            yield cause

    def min_age(self, min_age: Union[float, ConfigurableAge]) -> 'Signal':
        new_sig = copy.deepcopy(self)
        new_sig.min_age_value = min_age
        return new_sig

    def max_age(self, max_age: Union[float, ConfigurableAge]) -> 'Signal':
        new_sig = copy.deepcopy(self)
        new_sig.max_age_value = max_age
        return new_sig

    def detached(self) -> 'Signal':
        new_sig = copy.deepcopy(self)
        new_sig.detached_value = True
        return new_sig


class SignalRef(Signal):
    """
    Signal reference. Almost the same as a signal, except that
     it will not try to auto-discover it's module out of thread-local context
     (module_name will stay unfilled). Needed, because sometimes
     you need to reference a singal within a module scope
     without assigning that signal to the contextual module.
    """

    def __init__(self, name: str, *, min_age=0., max_age=5., detached=False):
        super().__init__(
            name,
            min_age=min_age,
            max_age=max_age,
            detached=detached,
            _skip_module_context=True)

    def id(self):
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
    _hash: int
    _allowed_causal_groups: Set[ICausalGroup]

    def __init__(self, *args):
        for arg in args:
            if not isinstance(arg, Signal):
                logger.error("Conjunct can only be constructed with Signals.")
                raise ValueError
        self._signals = set(args)
        self._hash = hash(tuple(sorted(sig.id() for sig in self._signals)))
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
            result = Conjunct(*(sig for sig in self._signals if not sig.detached_value))
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

    def referenced_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        yield from self._allowed_causal_groups

    def fulfilled_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        for child in self._signals:
            yield from child.fulfilled_causal_groups()

    def effect_not_caused(self, act: IActivation, group: ICausalGroup, effect: str) -> Generator['Signal', None, None]:
        # If the causal group is within the allowed, and one this conjuncts signals
        #  is within the forgone signals, then it is safe (?) to assume that all
        #  spikes from `group` from this conjunction must be rejected.
        if group in self._allowed_causal_groups and effect in self._signals:
            for child in self._signals:
                if child == effect and not child.spike:
                    yield from child.effect_not_caused(act, group, effect)
                    self._allowed_causal_groups = {sig.spike.causal_group() for sig in self._signals if sig.spike}
                    break

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

    def referenced_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        for child in self._conjunctions:
            yield from child.referenced_causal_groups()

    def fulfilled_causal_groups(self) -> Generator[ICausalGroup, None, None]:
        for child in self._conjunctions:
            yield from child.fulfilled_causal_groups()

    def effect_not_caused(self, act: IActivation, group: ICausalGroup, effect: str) -> Generator['Signal', None, None]:
        for child in self._conjunctions:
            yield from child.effect_not_caused(act, group, effect)

    def __str__(self):
        return " | ".join(map(lambda conjunct: conjunct.__str__(), self._conjunctions))
