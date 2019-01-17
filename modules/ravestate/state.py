# Ravestate State-related definitions

from typing import Callable, Optional, Any, Tuple, Union

from ravestate.constraint import Conjunct, Disjunct, Signal, s, Constraint
from ravestate.consumable import Consumable

from reggol import get_logger
logger = get_logger(__name__)


class StateActivationResult:
    """
    Base class for return values of state activation functions.
    """
    pass


class Delete(StateActivationResult):
    """
    Return an instance of this class, if the invoked state should be deleted.

    * `resign`: Set to true, if the state being deleted is due to
     it failing to execute, so a resignation is implied.
     This means, that the spikes that were allocated for it's activation
      may be re-used by another state.
    """
    def __init__(self, resign: bool=False):
        self.resign = resign


class Wipe(StateActivationResult):
    """
    Return an instance of this class, if context.wipe(signal) should be called,
     to ensure that there are no more active spikes for the state's signal.
    """
    pass


class Emit(StateActivationResult):
    """
    Return an instance of this class, if the invoked state's signal should be emitted.

    * `wipe`: Set to true, if context.wipe(signal) should be called before emit,
     to ensure that there is only one free spike for the given signal.
    """
    def __init__(self, wipe: bool=False):
        self.wipe = wipe


class Resign(StateActivationResult):
    """
    Return an instance of this class, if the state invocation should be regarded unsuccessful.
     This means, that the state's signal will not be emitted, and the spikes
     that were allocated for it's activation may be re-used by another state.
    """
    pass


class State:

    _signal: Signal
    signal_name: str
    write_props: Tuple
    read_props: Tuple
    constraint: Constraint
    constraint_: Constraint  # Updated by context, to add constraint causes to constraint
    module_name: str

    # Dummy resource which allows CausalGroups to track acquisitions
    #  for states that don't have any write-props.
    consumable: Consumable

    def __init__(self, *,
                 signal_name: Optional[str],
                 write: Union[str, Tuple[str]],
                 read: Union[str, Tuple[str]],
                 cond: Constraint,
                 action,
                 is_receptor: Optional[bool]=False):

        assert(callable(action))
        self.name = action.__name__
        self.consumable = Consumable(f"@{action.__module__}:{action.__name__}")

        # check to recognize states using old signal implementation
        if isinstance(cond, str):
            logger.error(f"Attempt to create state {self.name} which has a string as condition, not a constraint!")
            cond = None

        # catch the insane case
        if not len(read) and not cond and not is_receptor:
            logger.warning(
                f"The state `{self.name}` is not reading any properties, nor waiting for any signals. " +
                "It will never be activated!")

        # convert read/write properties to tuples
        if isinstance(write, str):
            write = (write,)
        if isinstance(read, str):
            read = (read,)

        # listen to default changed-signals if no signals are given.
        # convert triggers to disjunctive normal form.
        if not cond and len(read) > 0:
            cond = Disjunct(*list(Conjunct(s(f"{rprop_name}:changed")) for rprop_name in read))

        self.signal_name = signal_name
        self.write_props = write
        self.read_props = read
        self.constraint = cond
        self.constraint_ = cond
        self.action = action
        self.module_name = ""
        self._signal = None

    def __call__(self, context, *args, **kwargs) -> Optional[StateActivationResult]:
        args = (context,) + args
        return self.action(*args, **kwargs)

    def signal(self) -> Optional[Signal]:
        assert self.module_name
        if not self._signal and self.signal_name:
            self._signal = s(f"{self.module_name}:{self.signal_name}")
        return self._signal


def state(*, signal_name: Optional[str]="", write: tuple=(), read: tuple=(), cond: Constraint=None):
    """
    Decorator to declare a new state, which may emit a certain signal,
    write to a certain set of properties (calling write, push, pop),
    and read from certain properties (calling read).
    """
    def state_decorator(action):
        nonlocal signal_name, write, read, cond
        return State(signal_name=signal_name, write=write, read=read, cond=cond, action=action)
    return state_decorator
