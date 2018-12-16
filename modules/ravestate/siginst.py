# Ravestate class which encapsulates a single signal instance

from typing import List, Set, Dict, Optional
from ravestate.iactivation import IStateActivation, ISignalInstance
from threading import Lock
from collections import defaultdict
from ravestate.causalgroup import CausalGroup

from reggol import get_logger
logger = get_logger(__name__)


class SignalInstance(ISignalInstance):
    """
    This class encapsulates a single signal instance, to track
     ... it's consumption for different output properties
     ... it's offspring instances (causal group -> instances caused by this signal)
    """

    # Age of the instance in ticks
    _age: int

    # Name of the instance's signal
    _name: str

    # This signal instance's causal group. The causal group
    #  is shared by an instance's family (children/parents),
    #  and dictates which properties are still free to be written.
    _causal_group: CausalGroup

    # Offspring signals which were (partially) caused by this instance,
    #  with which consumed props are synced, and which are wiped if this
    #  instance is wiped.
    _offspring: Set['SignalInstance']

    # Parent instances, which are notified when this instance is wiped
    _parents: Set['SignalInstance']

    def __init__(self, *, signame: str, parents: Set['SignalInstance'], properties: Set[str]):
        """
        Construct a signal instance from a signal name and a list of causing parent signals.
        :param signame: Name of the signal which is represented by this signal instance
        :param parents: The parent signals which were involved in causing this signal,
         and with which unwritten properties will be synchronized: This signal instance's
         causal group will be inferred from the parents, and the parent's causal
         groups will be merged if they are different.
        :param properties: The set of property names from context,
         which are available for consumption.
        """
        self._name = signame
        self._offspring = set()
        self._parents = parents
        self._causal_group = next(iter(parents)).causal_group() if parents else CausalGroup(properties)
        self._suitors_per_property = {prop: set() for prop in properties}
        for parent in parents:
            parent.adopt(self)

    def name(self) -> str:
        """
        Returns the name if this signal instance's signal.
        """
        return self._name

    def causal_group(self) -> CausalGroup:
        """
        Get this signal instance's causal group.
        :return: This instances causal group. Should never be None.
        """
        return self._causal_group

    def adopt(self, child: 'SignalInstance') -> None:
        """
        Called in signal instance constructor, for instances which claim to be
         caused by this signal instance.
        :param child: The child to add to this signal instance's causal group.
        """
        self._offspring.add(child)
        if self.causal_group() != child.causal_group():
            with self.causal_group() as causal_parent, child.causal_group() as causal_child:
                causal_parent.merge(causal_child)

    def wiped(self, child: 'ISignalInstance') -> None:
        """
        Called by an offspring signal, to notify the extension instance
         that it was wiped, and should therefore be removed from the children set.
        :param child: The child to be forgotten.
        """
        if child not in self._offspring:
            logger.warning(f"Offspring {child.name()} requested to be removed from {self.name()}, but it's unfamiliar!")
            return
        self._offspring.remove(child)


    def wipe(self) -> None:
        # Wipe children. Copy set, because self._offspring will be manipulated during iteration.
        offspring = self._offspring.copy()
        for child in self._offspring:
            child.wipe()
        # Notify parents of their child's demise
        for parent in self._parents:
            parent.wiped(self)
        # Wipe from causal group
        with self.causal_group() as causal:
            causal.wipe(self)

    def tick(self) -> None:
        """
        Increment this signal instance's age by 1.
        """
        self._age += 1

    def age(self) -> int:
        """
        Obtain this signal instance's age (in ticks).
        """
        return self._age
