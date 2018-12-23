# Ravestate class which encapsualtes the activation of a single state
import copy

from ravestate import state
from ravestate import wrappers
from ravestate import icontext
from threading import Thread
from typing import Set

from ravestate.constraint import Signal, Constraint, s
from ravestate.iactivation import IActivation, ISignalInstance
from ravestate.siginst import SignalInstance


class Activation(IActivation):
    """
    Encapsulates the potential activation of a state. Tracks the fulfillment of
     certain state-defined constraints that are required before activation.
    """

    def __init__(self, st: state.State, ctx: icontext.IContext):
        self.name = st.signal_name()
        self.state_to_activate = st
        self.constraint: Constraint = copy.deepcopy(st.constraint)
        self.ctx = ctx
        self.args = []
        self.kwargs = {}

    def specificity(self):
        # TODO: Calculate specificity properly
        return 1.

    def write_props(self) -> Set[str]:
        pass

    def wiped(self, signal: ISignalInstance) -> None:
        pass

    def acquire(self, signal: SignalInstance) -> None:
        self.constraint.acquire(signal)

    def update(self) -> bool:  # Returns False if out of hope for activation
        pass

    def run(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        return Thread(target=self._run_private)

    def _run_private(self):
        context_wrapper = wrappers.ContextWrapper(self.ctx, self.state_to_activate)
        result = self.state_to_activate(context_wrapper, *self.args, **self.kwargs)
        if isinstance(result, state.Emit) and self.state_to_activate.signal:
            self.ctx.emit(s(self.state_to_activate.signal_name()))
        if isinstance(result, state.Delete):
            self.ctx.rm_state(st=self.state_to_activate)
        # TODO: Gather affected causal groups, call consumed(...) on each!
