# Ravestate class which encapsulates a single spike

from typing import Set
from ravestate.iactivation import ISpike
from ravestate.causal import CausalGroup

from reggol import get_logger
logger = get_logger(__name__)


class Spike(ISpike):
    """
    This class encapsulates a single spike, to track
     ... it's consumption for different output properties
     ... it's offspring instances (causal group -> spikes caused by this spike)
    """

    # Age of the spike in ticks
    _age: int

    # Name of the spike's signal
    _name: str

    # This spike's causal group. The causal group
    #  is shared by an spike's family (children/parents),
    #  and dictates which properties are still free to be written.
    _causal_group: CausalGroup

    # Offspring signals which were (partially) caused by this spike,
    #  with which consumed props are synced, and which are wiped if this
    #  spike is wiped.
    _offspring: Set['ISpike']

    # Parent instances, which are notified when this spike is wiped
    _parents: Set['Spike']

    def __init__(self, *, sig: str, parents: Set['Spike']=None, properties: Set[str]=None):
        """
        Construct a spike from a signal name and a list of causing parent signals.
        :param sig: Name of the signal which is represented by this spike
        :param parents: The parent signals which were involved in causing this signal,
         and with which unwritten properties will be synchronized: This spike's
         causal group will be inferred from the parents, and the parent's causal
         groups will be merged if they are different.
        :param properties: The set of property names from context,
         which are available for consumption.
        """
        if parents is None:
            parents = set()
        if properties is None:
            properties = set()
        self._name = sig
        self._age = 0
        self._offspring = set()
        self._parents = parents
        self._causal_group = next(iter(parents)).causal_group() if parents else CausalGroup(properties)
        self._suitors_per_property = {prop: set() for prop in properties}
        for parent in parents:
            parent.adopt(self)

    def __repr__(self):
        return f"Spike({self._name}, age={self._age})"

    def name(self) -> str:
        """
        Returns the name of this spike's signal.
        """
        return self._name

    def causal_group(self) -> CausalGroup:
        """
        Get this spike's causal group.
        :return: This instances causal group. Should never be None.
        """
        return self._causal_group

    def adopt(self, child: 'Spike') -> None:
        """
        Called in spike constructor, for instances which claim to be
         caused by this spike.
        :param child: The child to add to this spike's causal group.
        """
        self._offspring.add(child)
        if self.causal_group() != child.causal_group():
            with self.causal_group() as causal_parent, child.causal_group() as causal_child:
                causal_parent.merge(causal_child)

    def wiped(self, child: 'ISpike') -> None:
        """
        Called by an offspring signal, to notify the spike
         that it was wiped, and should therefore be removed from the children set.
        :param child: The child to be forgotten.
        """
        if child not in self._offspring:
            logger.warning(f"Offspring {child.name()} requested to be removed from {self.name()}, but it's unfamiliar!")
            return
        self._offspring.remove(child)

    def wipe(self, already_wiped_in_causal_group: bool=False) -> None:
        """
        Called either in Context run loop when the spike is found to be stale
         (with wiped_in_causal_group=True), or in Context.wipe(spike),
         or by parent (recursively).
        After this function is called, the spike should be cleaned up by GC.
        :param already_wiped_in_causal_group: Boolean which indicates, whether wiped(spike)
         must still be called on the group to make sure sure that no dangling references
         to the spike are maintained by any state activations.
        """
        # Wipe children. Copy set, because self._offspring will be manipulated during iteration.
        offspring = self._offspring.copy()
        for child in offspring:
            child.wipe(already_wiped_in_causal_group)
        # Notify parents of their child's demise
        for parent in self._parents:
            parent.wiped(self)
        self._parents.clear()
        # Wipe from causal group
        if not already_wiped_in_causal_group:
            with self.causal_group() as causal:
                causal.wiped(self)

    def has_offspring(self):
        """
        Called by CausalGroup.stale(spike).
        :return: True if the spike has active offspring, false otherwise.
        """
        return len(self._offspring) > 0

    def tick(self) -> None:
        """
        Increment this spike's age by 1.
        """
        self._age += 1

    def age(self) -> int:
        """
        Obtain this spike's age (in ticks).
        """
        return self._age
