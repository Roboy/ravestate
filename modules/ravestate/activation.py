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

    def write_props(self) -> Set[str]:
        """
        Return's the set of the activation's write-access property names.
        """
        return self.state_to_activate.write_props

    def specificity(self) -> float:
        """
        Returns the lowest specificity among the specificity values of the
         activation's conjunct constraints. The specificity for a single conjunction
         is calculated as the sum of it's component signal's specificities,
         which in turn is calculated as one over the signal's subscriber count.
        """
        return min(
            sum(self.ctx.signal_specificity(sig) for sig in conj.signals())
            for conj in self.constraint.conjunctions())

    def wiped(self, sig: ISignalInstance) -> None:
        """
        Notify the activation, that a certain signal instance is not available
         anymore, and should therefore not be referenced anymore by the activation.
        :param sig: The signal that should be forgotten by the activation
        """
        pass

    def eliminate(self, reacquire: bool=False) -> None:
        """
        Eliminate the activation, by making it reject all of it's referenced
         signal instances. This is called either by context when a state is deleted,
         or by this activation (with reacquire=True), if it gives in to
         activation pressure.
        :param reacquire: Flag which tells the function, whether for every rejected
         signal instance, the activation should hook into context for reacquisition
         of a replacement signal instance.
        """
        pass

    def signal_instances(self) -> Set[ISignalInstance]:
        """
        Called by causal group, to remove this activation's signal instances
         from it's internal activation candidate index once they are promised
         to this activation.
        """
        pass

    def acquire(self, signal: SignalInstance) -> bool:
        """
        Let the activation acquire a signal it is registered to be interested in.
        :param signal: The signal which should fulfill at least one of this activation's
         signal constraints.
        :return: Should return True.
        """
        return self.constraint.acquire(signal)

    def update(self) -> None:
        """
        Called once per tick on this activation, to give it a chance to activate
         itself, or auto-eliminate, or reject signal instances which have become too old.
        """
        pass

    def run(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        Thread(target=self._run_private).run()

    def _run_private(self):
        context_wrapper = wrappers.ContextWrapper(self.ctx, self.state_to_activate)
        result = self.state_to_activate(context_wrapper, *self.args, **self.kwargs)
        if isinstance(result, state.Emit) and self.state_to_activate.signal:
            self.ctx.emit(s(self.state_to_activate.signal_name()))
        if isinstance(result, state.Delete):
            self.ctx.rm_state(st=self.state_to_activate)
        # TODO: Gather affected causal groups, call consumed(...) on each!
