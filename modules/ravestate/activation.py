# Ravestate class which encapsulates the activation of a single state
import copy

from threading import Thread
from typing import Set, Optional, List, Dict, Any, Generator
from collections import defaultdict

from ravestate.icontext import IContext
from ravestate.constraint import Constraint
from ravestate.iactivation import IActivation, ISpike, ICausalGroup
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

    # Count how many activations were created per state
    _count_for_state: Dict[State, int] = defaultdict(int)

    death_clock: Optional[int]  # set once pressure() is called
    id: str  # _count_for_state[self.state_to_activate] from ctor time
    name: str
    state_to_activate: State
    constraint: Constraint
    ctx: IContext
    args: List
    kwargs: Dict
    spike_payloads: Dict[str, Any]
    parent_spikes: Set[Spike]
    consenting_causal_groups: Set[CausalGroup]
    pressuring_causal_groups: Set[ICausalGroup]

    def __init__(self, st: State, ctx: IContext):
        self.id = f"{st.name}#{Activation._count_for_state[st]}"
        Activation._count_for_state[st] += 1
        self.name = st.name
        self.state_to_activate = st
        self.constraint = copy.deepcopy(st.completed_constraint)
        self.ctx = ctx
        self.args = []
        self.kwargs = {}
        self.parent_spikes = set()
        self.consenting_causal_groups = set()
        self.death_clock = None
        self.pressuring_causal_groups = set()
        self.spike_payloads = dict()

    def __del__(self):
        logger.debug(f"Deleted {self}")

    def __repr__(self):
        return self.id + (f"[t-{self.death_clock}]" if self.death_clock is not None else "")

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
            return {self.state_to_activate.consumable.id()}

    def specificity(self) -> float:
        """
        Returns the lowest specificity among the specificity values of the
         activation's conjunct constraints. The specificity for a single conjunction
         is calculated as the sum of it's component signal's specificities,
         which in turn is calculated as one over the signal's subscriber count.
        """
        return min(
            sum(self.ctx.signal_specificity(sig) for sig in conj.signals())
            for conj in self.constraint.conjunctions()) * self.state_to_activate.get_current_weight()

    def dereference(self, *, spike: Optional[ISpike]=None, reacquire: bool=False, reject: bool=False, pressured: bool=False) -> None:
        """
        Notify the activation, that a single or all spike(s) are not available
         anymore, and should therefore not be referenced anymore by the activation.
        This is called by ... <br>
        ... context when a state is deleted. <br>
        ... causal group, when a referenced signal was consumed for a required property. <br>
        ... causal group, when a referenced signal was wiped. <br>
        ... this activation (with reacquire=True and pressured=True), if it gives in to activation pressure.

        * `spike`: The spike that should be forgotten by the activation, or
         none, if all referenced spikes should be forgotten.

        * `reacquire`: Flag which tells the function, whether for every rejected
         spike, the activation should hook into context for reacquisition
         of a replacement spike.

        * `reject`: Flag which controls, whether de-referenced spikes
         should be explicitely rejected through their causal groups.

        * `pressured`: Flag which controls, whether de-referencing should only occur
         for spikes of causal groups in the pressuring_causal_groups set.
        """
        unreferenced = self.constraint.dereference(spike=spike, causal_groups=self.pressuring_causal_groups)
        message = ""
        for sig_to_reacquire, dereferenced_spike in unreferenced:
            message += " " + repr(dereferenced_spike)
            if reacquire:
                self.ctx.reacquire(self, sig_to_reacquire)
            if reject and dereferenced_spike:
                with dereferenced_spike.causal_group() as cg:
                    cg.rejected(dereferenced_spike, self, reason=0)
        if message:
            logger.debug(f"Dereferenced {self} from" + message)

    def acquire(self, spike: Spike) -> bool:
        """
        Let the activation acquire a signal it is registered to be interested in.

        * `spike`: The signal which should fulfill at least one of this activation's
         signal constraints.

        **Returns:** Should return True.
        """
        if self.death_clock is not None:
            self._reset_death_clock()
        return self.constraint.acquire(spike, self)

    def secs_to_ticks(self, seconds: float) -> int:
        """
        Convert seconds to an equivalent integer number of ticks,
         given this activation's tick rate.

        * `seconds`: Seconds to convert to ticks.

        **Returns:** An integer tick count.
        """
        return self.ctx.secs_to_ticks(seconds)

    def pressure(self, give_me_up: ICausalGroup):
        """
        Called by CausalGroup, to pressure the activation to
         make a decision on whether it is going to retain a reference
         to the given spike, given that there is a lower-
         specificity activation which is ready to run.

        * `give_me_up`: Causal group that wishes to be de-referenced by this activation.
        """
        if self.death_clock is None:
            self._reset_death_clock()
        self.pressuring_causal_groups = {causal for causal in self.pressuring_causal_groups} | {give_me_up}

    def is_pressured(self):
        """
        Called by context, to figure out whether the activation is pressured,
         and therefore the idle:bored signal should be emitted.
        """
        return self.death_clock is not None

    def spiky(self) -> bool:
        """
        Returns true, if the activation has acquired any spikes at all.

        **Returns:** True, if any of this activation's constraint's
         signal is referencing a spike.
        """
        return any(self.spikes())

    def spikes(self) -> Generator[Spike, None, None]:
        """
        Returns iterable for all the spikes currently acquired by the activation.
        """
        return (sig.spike for sig in self.constraint.signals() if sig.spike)

    def update(self) -> bool:
        """
        Called once per tick on this activation, to give it a chance to activate
         itself, or auto-eliminate, or reject spikes which have become too old.

        **Returns:** True, if the target state is activated and teh activation be forgotten,
         false if needs further attention in the form of updates() by context in the future.
        """
        # Update constraint, reacquire for rejected spikes
        for spike in self.constraint.update(self):
            self.ctx.reacquire(self, spike)

        # Iterate over fulfilled conjunctions and look to activate with one of them
        for conjunction in self.constraint.conjunctions():
            if not conjunction.evaluate():
                continue

            # Death is cheated if the activation is fulfilled.
            self.death_clock = None

            # Ask each spike's causal group for activation consent
            spikes_for_conjunct = set((sig.spike, sig.detached) for sig in conjunction.signals())
            consenting_causal_groups = set()
            all_consented = True
            for spike, detached in spikes_for_conjunct:
                if detached:
                    continue
                cg: CausalGroup = spike.causal_group()
                if cg not in consenting_causal_groups:
                    with cg:
                        if cg.consent(self):
                            consenting_causal_groups.add(cg)
                        else:
                            all_consented = False
                            break  # break spike iteration
            if not all_consented:
                continue

            # Gather payloads for all spikes
            self.spike_payloads = {
                spike.name(): spike.payload()
                for spike, _ in spikes_for_conjunct if spike.payload()}

            # Notify all consenting causal groups that activation is going forward
            for cg in consenting_causal_groups:
                with cg:
                    cg.activated(self)

            # Remove references between causal groups <-> self
            for signal in self.constraint.signals():
                if signal.spike:
                    with signal.spike.causal_group() as cg:
                        cg.rejected(signal.spike, self, reason=2)
                    signal.spike = None

            # Withdraw from context for all (unfulfilled) signals (there might
            #  be some unfulfilled conjunctions next to the fulfilled one).
            for sig in self.constraint.signals():
                self.ctx.withdraw(self, sig)

            # Remember spikes/causal-groups for use in activation
            if self.state_to_activate.emit_detached:
                self.parent_spikes = set()
            else:
                self.parent_spikes = {spike for spike, detached in spikes_for_conjunct if not detached}
            self.consenting_causal_groups = consenting_causal_groups

            # Run activation
            self.run()

            # Do not further iterate over candidate conjunctions
            return True

        # Update auto-elimination countdown
        if self.death_clock is not None:
            if self.death_clock <= 0:
                self.death_clock = None
                self.dereference(reacquire=True, reject=True, pressured=True)
                logger.info(f"Eliminated {self} from {self.pressuring_causal_groups}.")
                self.pressuring_causal_groups = set()
            else:
                self.death_clock -= 1

        return False

    def run(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        logger.info(f"Activating {self}")
        Thread(target=self._run_private).start()

    def _reset_death_clock(self):
        self.death_clock = self.ctx.lowest_upper_bound_eta(set(
            sig for sig in self.constraint.signals() if not sig.spike))

    def _unique_consenting_causal_groups(self) -> Set[CausalGroup]:
        # if a signal was emitted by this activation, the consenting
        #  causal groups will be merged. this must be respected -> recreate the set.
        return {group for group in self.consenting_causal_groups}

    def _run_private(self):
        context_wrapper = ContextWrapper(ctx=self.ctx,
                                         st=self.state_to_activate,
                                         spike_parents=self.parent_spikes,
                                         spike_payloads=self.spike_payloads)

        # Run state function
        try:
            result = self.state_to_activate(context_wrapper, *self.args, **self.kwargs)
        except Exception as e:
            logger.error(f"An exception occurred while activating {self}: {e}")
            result = Resign()

        # Process state function result
        if isinstance(result, Emit):
            if self.state_to_activate.signal():
                self.ctx.emit(
                    self.state_to_activate.signal(),
                    parents=self.parent_spikes,
                    wipe=result.wipe)
            else:
                logger.error(f"Attempt to emit spike from state {self.name}, which does not specify a signal name!")

        elif isinstance(result, Wipe):
            if self.state_to_activate.signal():
                self.ctx.wipe(self.state_to_activate.signal())
            else:
                logger.error(f"Attempt to wipe spikes from state {self.name}, which does not specify a signal name!")

        elif isinstance(result, Delete):
            self.ctx.rm_state(st=self.state_to_activate)
            if result.resign:
                for cg in self._unique_consenting_causal_groups():
                    with cg:
                        cg.resigned(self)
                self.state_to_activate.activated.release()
                return

        elif isinstance(result, Resign):
            for cg in self._unique_consenting_causal_groups():
                with cg:
                    cg.resigned(self)
            self.state_to_activate.activated.release()
            return

        # Let participating causal groups know about consumed properties
        for cg in self._unique_consenting_causal_groups():
            with cg:
                cg.consumed(self.resources())

        self.state_to_activate.activation_finished()
