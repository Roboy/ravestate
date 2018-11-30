# Ravestate class which encapsualtes the activation of a single state
import copy

from ravestate import state
from ravestate import wrappers
from ravestate import icontext
from threading import Thread

from ravestate.constraint import Signal, Constraint, s


class StateActivation:

    def __init__(self, st: state.State, ctx: icontext.IContext):
        self.state_to_activate = st
        self.unfulfilled: Constraint = copy.deepcopy(st.triggers)
        self.ctx = ctx
        self.args = []
        self.kwargs = {}

    def specificity(self):
        # TODO: Calculate specificity properly
        return 1.

    def notify_signal(self, signal: Signal):
        self.unfulfilled.set_signal_true(signal)
        return 1 if self.unfulfilled.evaluate() else 0

    def run(self, args=(), kwargs={}):
        self.args = args
        self.kwargs = kwargs
        return Thread(target=self._run_private)

    def _run_private(self):
        context_wrapper = wrappers.ContextWrapper(self.ctx, self.state_to_activate)
        result = self.state_to_activate(context_wrapper, self.args, self.kwargs)
        if isinstance(result, state.Emit) and self.state_to_activate.signal:
            self.ctx.emit(s(self.state_to_activate.signal_name()))
        if isinstance(result, state.Delete):
            self.ctx.rm_state(st=self.state_to_activate)
