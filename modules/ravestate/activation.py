# Ravestate class which encapsualtes the activation of a single state
import copy

from threading import Thread
from typing import Set, Optional, List, Dict

from ravestate.icontext import IContext
from ravestate.constraint import Constraint, s
from ravestate.iactivation import IActivation, ISpike
from ravestate.spike import Spike
from ravestate.causal import CausalGroup
from ravestate.state import State, Emit, Delete, Resign, Wipe
from ravestate.wrappers import ContextWrapper

from reggol import get_logger
logger = get_logger(__name__)


class Activation(IActivation):
    """
    Encapsulates the potential activation of a state. Tracks the collection
     of Spikes to fulfill of the state-defined activation constraints.
    """

    name: str
    state_to_activate: State
    constraint: Constraint
    ctx: IContext
    args: List
    kwargs: Dict
    spikes: Set[Spike]
    consenting_causal_groups: Set[CausalGroup]

    def __init__(self, st: State, ctx: IContext):
        self.name = st.name
        self.state_to_activate = st
        self.constraint = copy.deepcopy(st.constraint_)
        self.ctx = ctx
        self.args = []
        self.kwargs = {}
        self.spikes = set()
        self.consenting_causal_groups = set()

    def __del__(self):
        logger.debug(f"Deleted {self}")

    def __repr__(self):
        return f"Activation(st={self.name})"

    def resources(self) -> Set[str]:
        """
        Return's the set of the activation's write-access property names.
        """
        if self.state_to_activate.write_props:
            return set(self.state_to_activate.write_props)
        else:
            # Return a dummy resource that will be "consumed" by the activation.
            #  This allows CausalGroup to track the spike acquisitions for this
            #  activation, and make sure that a single spike cannot activate
            #  multiple activations for a write-prop-less state.
            return {self.state_to_activate.consumable.fullname()}

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

    def dereference(self, *, spike: Optional[ISpike]=None, reacquire: bool=False, reject: bool=False) -> None:
        """
        Notify the activation, that a single or all spike(s) are not available
         anymore, and should therefore not be referenced anymore by the activation.
        This is called by ...
         ... context when a state is deleted.
         ... causal group, when a referenced signal was consumed for a required property.
         ... causal group, when a referenced signal was wiped.
         ... this activation (with reacquire=True), if it gives in to activation pressure.
        :param spike: The spike that should be forgotten by the activation, or
         none, if all referenced spikes should be forgotten.
        :param reacquire: Flag which tells the function, whether for every rejected
         spike, the activation should hook into context for reacquisition
         of a replacement spike.
        :param reject: Flag which controls, whether de-referenced spikes
         should be explicitely rejected through their causal groups.
        """
        for sig_to_reacquire, dereferenced_instance in self.constraint.dereference(spike):
            if reacquire:
                self.ctx.reacquire(self, sig_to_reacquire)
            if reject and dereferenced_instance:
                with dereferenced_instance.causal_group() as cg:
                    cg.rejected(dereferenced_instance, self)

    def acquire(self, spike: Spike) -> bool:
        """
        Let the activation acquire a signal it is registered to be interested in.
        :param spike: The signal which should fulfill at least one of this activation's
         signal constraints.
        :return: Should return True.
        """
        return self.constraint.acquire(spike, self)

    def secs_to_ticks(self, seconds: float) -> int:
        """
        Convert seconds to an equivalent integer number of ticks,
         given this activation's tick rate.
        :param seconds: Seconds to convert to ticks.
        :return: An integer tick count.
        """
        return self.ctx.secs_to_ticks(seconds)

    def update(self) -> bool:
        """
        Called once per tick on this activation, to give it a chance to activate
         itself, or auto-eliminate, or reject spikes which have become too old.
        :return: True, if the target state is activated and teh activation be forgotten,
         false if needs further attention in the form of updates() by context in the future.
        """
        # Update constraint, reacquire for rejected spikes
        for spike in self.constraint.update(self):
            self.ctx.reacquire(self, spike)

        # Iterate over fulfilled conjunctions and look to activate with one of them
        for conjunction in self.constraint.conjunctions():
            if not conjunction.evaluate():
                continue

            # Ask each spike's causal group for activation consent
            spikes = set((sig.spike, sig.detached) for sig in conjunction.signals())
            consenting_causal_groups = set()
            all_consented = True
            for spike, _ in spikes:
                cg: CausalGroup = spike.causal_group()
                if cg not in consenting_causal_groups:
                    with cg:
                        if cg.pressure_activation(self):
                            consenting_causal_groups.add(cg)
                        else:
                            all_consented = False
                            break  # break spike iteration
            if not all_consented:
                continue

            # Notify all consenting causal groups that activation is going forward
            for cg in consenting_causal_groups:
                with cg:
                    cg.activated(self)

            # Remove self from all causal groups
            for spike, _ in spikes:
                with spike.causal_group() as cg:
                    cg.rejected(spike, self)

            # Make sure that constraint doesn't hold any unneeded references to spike
            for _ in self.constraint.dereference():
                pass

            # Withdraw from context for all (unfulfilled) signals (there might
            #  be some unfulfilled conjunctions next to the fulfilled one).
            for sig in self.constraint.signals():
                self.ctx.withdraw(self, sig)

            # Remember spikes/causal-groups for use in activation
            self.spikes = {spike for spike, detached in spikes if not detached}
            self.consenting_causal_groups = consenting_causal_groups

            # Run activation
            self.run()

            # Do not further iterate over candidate conjunctions
            return True
        return False

    def run(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        Thread(target=self._run_private).start()

    def _unique_consenting_causal_groups(self) -> Set[CausalGroup]:
        # if a signal was emitted by this activation, the consenting
        #  causal groups will be merged. this must be respected -> recreate the set.
        return {group for group in self.consenting_causal_groups}

    def _run_private(self):
        context_wrapper = ContextWrapper(ctx=self.ctx, st=self.state_to_activate, spike_parents=self.spikes)

        # Run state function
        result = self.state_to_activate(context_wrapper, *self.args, **self.kwargs)

        # Process state function result
        if isinstance(result, Emit):
            if self.state_to_activate.signal():
                if result.wipe:
                    self.ctx.wipe(self.state_to_activate.signal())
                self.ctx.emit(
                    self.state_to_activate.signal(),
                    parents=self.spikes)
            else:
                logger.error(f"Attempt to emit spike from state {self.name}, which does not specify a signal name!")

        elif isinstance(result, Wipe):
            if self.state_to_activate.signal():
                self.ctx.wipe(self.state_to_activate.signal())
            else:
                logger.error(f"Attempt to wipe spikes from state {self.name}, which does not specify a signal name!")

        elif isinstance(result, Delete):
            self.ctx.rm_state(st=self.state_to_activate)

        elif isinstance(result, Resign):
            for cg in self._unique_consenting_causal_groups():
                with cg:
                    cg.resigned(self)
            return

        # Let participating causal groups know about consumed properties
        for cg in self._unique_consenting_causal_groups():
            with cg:
                cg.consumed(self.resources())
