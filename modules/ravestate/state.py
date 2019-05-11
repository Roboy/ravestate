# Ravestate State-related definitions

from typing import Optional, Tuple, Union, Set
from threading import Semaphore, Lock

from ravestate.property import Property
from ravestate.threadlocal import ravestate_thread_local
from ravestate.constraint import Conjunct, Disjunct, Signal, SignalRef, Constraint
from ravestate.consumable import Consumable

from reggol import get_logger
logger = get_logger(__name__)


class _StateActivationResult:
    """
    Base class for return values of state activation functions.
    """
    pass


class Delete(_StateActivationResult):
    """
    Return an instance of this class, if the invoked state should be deleted.

    * `resign`: Set to true, if the state being deleted is due to
     it failing to execute, so a resignation is implied.
     This means, that the spikes that were allocated for it's activation
      may be re-used by another state.
    """
    def __init__(self, resign: bool=False):
        self.resign = resign


class Wipe(_StateActivationResult):
    """
    Return an instance of this class, if context.wipe(signal) should be called,
     to ensure that there are no more active spikes for the state's signal.
    """
    pass


class Emit(_StateActivationResult):
    """
    Return an instance of this class, if the invoked state's signal should be emitted.

    * `wipe`: Set to true, if context.wipe(signal) should be called before emit,
     to ensure that there is only one free spike for the given signal.
    """
    def __init__(self, wipe: bool=False):
        self.wipe = wipe


class Resign(_StateActivationResult):
    """
    Return an instance of this class, if the state invocation should be regarded unsuccessful.
     This means, that the state's signal will not be emitted, and the spikes
     that were allocated for it's activation may be re-used by another state.
    """
    pass


class State:
    """
    Class which encapsulates a single application state function.
    Do not use this class - instead use the `@state` decorator.
    """

    signal: Signal
    write_props: Set[str]
    read_props: Set[str]
    constraint: Constraint
    emit_detached: bool
    cooldown: float
    weight: float
    is_receptor: bool

    module_name: str                  # The module which this state belongs to
    completed_constraint: Constraint  # Updated by context, to add constraint causes to constraint
    activated: Semaphore              # Semaphore which counts finished activations
    lock: Lock                        # Mutex to lock access to the current weight/cooldown state
    current_weight: float             # Current weight, as affected by cooldown

    # Dummy resource which allows CausalGroups to track spike acquisitions
    #  for states that don't have any write-props.
    consumable: Consumable

    def __init__(self, *,
                 signal: Optional[Signal],
                 write: Union[Property, str, Tuple[Property, str]],
                 read: Union[Property, str, Tuple[Property, str]],
                 cond: Optional[Constraint],
                 action,
                 is_receptor: bool = False,
                 emit_detached: bool = False,
                 weight: float = 1.,
                 cooldown: float = 0.):

        assert(callable(action))
        self.name = action.__name__
        if not is_receptor:
            self.consumable = Consumable(f"@{action.__name__}")

        # check to recognize states using old signal implementation
        if isinstance(cond, str):
            logger.error(f"Attempt to create state {self.name} which has a string as condition, not a constraint!")
            cond = None

        # normalize read/write properties to sets of strings
        if isinstance(write, Property) or isinstance(write, str):
            write = (write,)
        if isinstance(read, Property) or isinstance(write, str):
            read = (read,)
        write: Set[str] = set(
            write_prop.id() if isinstance(write_prop, Property) else write_prop
            for write_prop in write)
        read: Set[str] = set(
            read_prop.id() if isinstance(read_prop, Property) else read_prop
            for read_prop in read)

        # catch the insane case
        if not len(read) and not cond and not is_receptor:
            logger.warning(
                f"The state `{self.name}` is not reading any properties, nor waiting for any signals. " +
                "It will never be activated!")

        # listen to default changed-signals if no signals are given.
        # convert triggers to disjunctive normal form.
        if not cond and len(read) > 0:
            cond = Disjunct(*(Conjunct(SignalRef(f"{read_prop}:changed")) for read_prop in read))

        self.signal = signal
        self.write_props = write
        self.read_props = read
        self.constraint = cond
        self.completed_constraint = cond
        self.action = action
        self.module_name = ""
        self.emit_detached = emit_detached
        self.activated = Semaphore(0)
        self.weight = self.current_weight = weight
        self.cooldown = cooldown
        self.is_receptor = is_receptor
        self.lock = Lock()

        # add state to module in current `with Module(...)` clause
        module_under_construction = getattr(ravestate_thread_local, 'module_under_construction', None)
        if module_under_construction:
            module_under_construction.add(self)

    def __call__(self, context, *args, **kwargs) -> Optional[_StateActivationResult]:
        args = (context,) + args
        return self.action(*args, **kwargs)

    def get_read_props_ids(self) -> Set[str]:
        return self.read_props

    def get_write_props_ids(self) -> Set[str]:
        return self.write_props

    def get_all_props_ids(self) -> Set[str]:
        return self.read_props | self.write_props

    def update_weight(self, seconds_passed: float):
        """
        Called once per tick by context, where #seconds_passed is
         one over the context's tick rate. The state will use this
         function, to update it's actual weight, given it's cooldown
         period and last activation
        """
        if self.cooldown <= .0 or self.current_weight >= self.weight:
            return
        with self.lock:
            self.current_weight += seconds_passed/self.cooldown * self.weight
            if self.current_weight > self.weight:
                self.current_weight = self.weight

    def get_current_weight(self):
        """
        Called by activation to obtain the weight factor for this
         state's constraint's specificity.
        """
        with self.lock:
            return self.current_weight

    def activation_finished(self):
        """
        Called by a running activation for this state, once it is about to
         exit it's dedicated thread.
        """
        with self.lock:
            self.activated.release()
            if self.cooldown > .0:
                # Reset weight, it will be raised to it's original level
                # over the course of repeated update_weight() calls.
                self.current_weight = .0

    def wait(self, timeout=5.):
        """
        Wait for the state's activation function to be run at least once.

        * `timeout`: Timeout after which this function should return False,
         if the activation is not occurring.

        __Return:__ True if the activation has been invoked at least once
         since the last call to this function, false if invocation does not
         occur after #timeout seconds.
        """
        return self.activated.acquire(timeout=timeout)


def state(*,
          signal: Optional[Signal] = "",
          write: Tuple[Property] = (),
          read: Tuple[Property] = (),
          cond: Constraint = None,
          emit_detached: bool = False,
          weight: float = 1.,
          cooldown: float = 0.):

    """
    Decorator to declare a new state, which may emit a certain signal,
    write to a certain set of properties (calling write, push, pop),
    and read from certain properties (calling read).

    _Example (Module that outputs "Don't Panic" after startup):_
    ```python
    with Module(name="my_module"):
        @state(cond=startup())
        def after_startup(context, write=OUTPUT_PROPERTY):
            context[OUTPUT_PROPERTY] = "Don't Panic"
    ```
    """
    def state_decorator(action):
        nonlocal signal, write, read, cond
        return State(
            signal=signal,
            write=write,
            read=read,
            cond=cond,
            action=action,
            emit_detached=emit_detached,
            weight=weight,
            cooldown=cooldown)
    return state_decorator
