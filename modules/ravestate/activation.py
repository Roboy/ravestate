# Ravestate class which encapsualtes the activation of a single state
import copy

from threading import Thread
from typing import Set, Optional, List, Dict

from ravestate.icontext import IContext
from ravestate.constraint import Constraint, s
from ravestate.iactivation import IActivation, ISignalInstance
from ravestate.siginst import SignalInstance
from ravestate.causalgroup import CausalGroup
from ravestate.state import State, Emit, Delete
from ravestate.wrappers import ContextWrapper

from reggol import get_logger
logger = get_logger(__name__)


class Activation(IActivation):
    """
    Encapsulates the potential activation of a state. Tracks the fulfillment of
     certain state-defined constraints that are required before activation.
    """

    name: str
    state_to_activate: State
    constraint: Constraint
    ctx: IContext
    args: List
    kwargs: Dict
    signal_instances: Set[SignalInstance]
    consenting_causal_groups: Set[CausalGroup]

    def __init__(self, st: State, ctx: IContext):
        self.name = st.name
        self.state_to_activate = st
        self.constraint = copy.deepcopy(st.constraint)
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

    def dereference(self, *, sig: Optional[ISignalInstance]=None, reacquire: bool=False) -> None:
        """
        Notify the activation, that a single or all signal instance(s) are not available
         anymore, and should therefore not be referenced anymore by the activation.
        This is called by ...
         ... context when a state is deleted.
         ... causal group, when a referenced signal was consumed for a required property.
         ... causal group, when a referenced signal was wiped.
         ... this activation (with reacquire=True), if it gives in to activation pressure.
        :param sig: The signal that should be forgotten by the activation, or
         none, if all referenced signal instances should be forgotten.
        :param reacquire: Flag which tells the function, whether for every rejected
         signal instance, the activation should hook into context for reacquisition
         of a replacement signal instance.
        """
        for sig_to_reacquire in self.constraint.dereference(sig):
            if reacquire:
                self.ctx.reacquire(self, sig_to_reacquire)

    def acquire(self, signal: SignalInstance) -> bool:
        """
        Let the activation acquire a signal it is registered to be interested in.
        :param signal: The signal which should fulfill at least one of this activation's
         signal constraints.
        :return: Should return True.
        """
        return self.constraint.acquire(signal, self)

    def update(self) -> None:
        """
        Called once per tick on this activation, to give it a chance to activate
         itself, or auto-eliminate, or reject signal instances which have become too old.
        """
        # Update constraint
        signals_to_reacquire = self.constraint.update()
        # Reacquire for rejected signal instances
        for sig in signals_to_reacquire:
            self.ctx.reacquire(self, sig)
        # Iterate over fulfilled conjunctions and look to activate with one of them
        for conjunction in self.constraint.conjunctions():
            if not conjunction.evaluate():
                continue
            # Ask each signal instance's causal group for activation consent
            signal_instances = set(sig.signal_instance for sig in conjunction.signals())
            consenting_causal_groups = set()
            all_consented = True
            for sig in signal_instances:
                cg: CausalGroup = sig.causal_group()
                if cg not in consenting_causal_groups:
                    with cg:
                        if cg.pressure_activation(self):
                            consenting_causal_groups.add(cg)
                        else:
                            all_consented = False
                            break  # break signal instance iteration
            if not all_consented:
                continue
            # Notify all consenting causal groups that activation is going forward
            for cg in consenting_causal_groups:
                with cg:
                    cg.activated(self)
            # Remove self from all causal groups
            for sig in signal_instances:
                with sig.causal_group() as cg:
                    cg.rejected(sig, self)
            # Make sure that constraint doesn't hold any unneeded references to signal inst.
            for _ in self.constraint.dereference(None):
                pass
            # Withdraw from context for all (unfulfilled) signals (there might
            #  be some unfulfilled conjunctions next to the fulfilled one).
            for sig in self.constraint.signals():
                self.ctx.withdraw(self, sig)
            # Remember sig-instances/causal-groups for use in activation
            self.signal_instances = signal_instances
            self.consenting_causal_groups = consenting_causal_groups
            # Run activation
            self.run()
            # Do not further iterate over candidate conjunctions
            break

    def run(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        Thread(target=self._run_private).run()

    def _run_private(self):
        context_wrapper = ContextWrapper(self.ctx, self.state_to_activate)
        result = self.state_to_activate(context_wrapper, *self.args, **self.kwargs)
        if isinstance(result, Emit):
            if self.state_to_activate.signal:
                self.ctx.emit(
                    s(self.state_to_activate.signal_name()),
                    parents=self.signal_instances)
            else:
                logger.error(f"Attempt to emit from state {self.name}, which does not specify a signal name!")
        if isinstance(result, Delete):
            self.ctx.rm_state(st=self.state_to_activate)
        for cg in self.consenting_causal_groups:
            with cg:
                cg.consumed(self.state_to_activate.write_props)
