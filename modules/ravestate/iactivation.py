# Interface of state activation towards spike
# Spike interface towards state activation

from typing import Set, Optional, Generator


class ISpike:
    """
    Base interface class for spikes.
    """

    def name(self) -> str:
        """
        Returns the name of this spike's signal.
        """
        pass

    def wipe(self, already_wiped_in_causal_group: bool=False) -> None:
        """
        Called either in Context run loop when the spike is found to be stale
         (with wiped_in_causal_group=True), or in Context.wipe(signal_inst),
         or by parent (recursively).
        After this function is called, the spike should be cleaned up by GC.

        * `already_wiped_in_causal_group`: Boolean which indicates, whether wiped(signal_inst)
         must still be called on the group to make sure sure that no dangling references
         to the spike are maintained by any state activations.
        """
        pass

    def has_offspring(self):
        """
        Called by CausalGroup.stale(spike).

        **Returns:** True if the spike has active offspring, false otherwise.
        """
        pass

    def offspring(self) -> Generator['Spike', None, None]:
        """
        Recursively yields this spike's offspring and it's children's offspring.

        **Returns:** All of this spike's offspring spikes.
        """
        yield None


class ICausalGroup:
    """
    Base class for causal group
    """
    pass


class IActivation:
    """
    Base interface class for state activations.
    """

    name: str

    def resources(self) -> Set[str]:
        """
        Return's the set of the activation's write-access property names.
        """
        pass

    def specificity(self) -> float:
        """
        Returns the lowest specificity among the specificity values of the
         activation's conjunct contraints. The specificity for a single conjunction
         is calculated as the sum of it's component signal's specificities,
         which in turn is calculated as one over the signal's subscriber count.
        """
        pass

    def dereference(self, *, spike: Optional[ISpike]=None, reacquire: bool=False, reject: bool=False, pressured: bool=False) -> None:
        """
        Notify the activation, that a single or all spike(s) are not available
         anymore, and should therefore not be referenced anymore by the activation.
        This is called by ... <br>
        ... context when a state is deleted. <br>
        ... causal group, when a referenced signal was consumed for a required property. <br>
        ... causal group, when a referenced signal was wiped. <br>
        ... this activation (with reacquire=True), if it gives in to activation pressure.

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
        pass

    def pressure(self, give_me_up: ICausalGroup):
        """
        Called by CausalGroup, to pressure the activation to
         make a decision on whether it is going to retain a reference
         to the given spike, given that there is a lower-
         specificity activation which is ready to run.

        * `give_me_up`: Causal group that wishes to be de-referenced by this activation.
        """
        pass

    def secs_to_ticks(self, seconds: float) -> int:
        """
        Convert seconds to an equivalent integer number of ticks,
         given this activation's context's tick rate.

        * `seconds`: Seconds to convert to ticks.

        **Returns:** An integer tick count.
        """
        pass
