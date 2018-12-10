# Ravestate class which encapsulates a single signal instance

from typing import List, Set, Dict, Optional
from ravestate.iactivation import IStateActivation, ISignalInstance
from threading import Lock

from reggol import get_logger
logger = get_logger(__name__)


class CausalGroup:
    """
    Class which represents a causal group graph of signal parent/offspring
     instances (a "supersignal"). These must synchronize wrt/ their (un)written
     properties and state activation candidates, such that they don't cause output races.
    """
    # Lock for the whole causal group
    _lock: Lock

    # Set of property names for which no writing state has been
    #  activated yet within this causal group.
    #
    # This is different from _refcount_per_signal_per_activation_per_property:
    #
    #  -> _unwritten_props might lack properties that are only
    #   *temporarily* unavailable, because a writing state is still running,
    #   and might still resign, which would make that state's write prop's
    #   available again.
    #
    #  -> it which case it is useful to still see the other
    #   state activation candidates for those props in _suitors_per_property.
    #
    # Also, _refcount_per_signal_per_activation_per_property needs to be as
    #  sparse as possible to optimise performance.
    #
    _unwritten_props: Set[str]

    # Refcount per state activations per signal instance per property name.
    #  Refcount, because one activation may hold multiple references to one signal for multiple
    #  differently timed constraints. We essentially keep a {(propname, activation, signal, refcount)}
    #  index structure, that will be necessary for the following use-cases:
    #
    #  * Determining whether an activation is the most specific for a certain property (O(i * log p))
    #    -> Gather state activations per instance per property, sort by specificity
    #  * Determining whether a signal is stale (no more activation references) (O(n)) -> Omitted per SignalInstance.refs
    #
    #  Container operations:
    #
    #  * Inserting a signal instance / state activation: O(log n)
    #  * Removing a state activation: O(log n)
    #  * Wiping a signal instance: O(p)
    #
    _refcount_per_signal_per_activation_per_property: Dict[str, Dict[ISignalInstance, Dict[IStateActivation, int]]]

    def __init__(self, properties: Set[str]):
        self._lock = Lock()
        self._unwritten_props = properties.copy()

    def merge(self, other: 'CausalGroup'):
        pass


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
            parent.add_child(self)

    def causal_group(self) -> CausalGroup:
        """
        Get this signal instance's causal group.
        :return: This instances causal group. Should never be None.
        """
        return self._causal_group

    def add_child(self, child: 'SignalInstance') -> None:
        """
        Called in signal instance constructor, for instances which claim to be
         caused by this signal instance.
        :param child: The child to add to this signal instance's causal group.
        """
        self._offspring.add(child)
        child.causal_group().merge(self.causal_group())

    def acquired(self, acquired_by: IStateActivation) -> None:
        """
        Called by StateActivation to notify the signal instance, that
         it is being referenced by an activation constraint.
        :param acquired_by: State activation instance, which is interested in this property.
        """
        for prop in acquired_by.write_props():
            if prop not in self._unwritten_props:
                logger.error(f"Attempt to acquire signal instance {self._name} twice for property {prop}!")
            self._suitors_per_property[prop].add(acquired_by)

    def rejected(self, rejected_by: IStateActivation) -> None:
        """
        Called by a state activation, to notify this signal instance
         that it is no longer being referenced for the given state's write props.
        This may be either because the state activation resigned,
         or because this signal instance got too old.
        :param rejected_by: State activation instance, which is no longer
         interested in this property.
        """
        for prop in rejected_by.write_props():
            self._suitors_per_property[prop].remove(rejected_by)
            if not self._suitors_per_property[prop]:
                del self._suitors_per_property[prop]

    def consent_consumption(self, consumer: IStateActivation) -> bool:
        """
        Called by constraint, to inquire whether this instance would happily
         be consumed for the given state activation's properties.
        This will be called periodically on the instance by state activations
         that are ready to go. Therefore, a False return value from this
         function is never a final judgement (more like a "maybe later").
        :param consumer: The state activation which would like to consume
         this instance for it's write props.
        :return: True if this instance agrees to proceeding with the given consumer
         for the consumer's write props, False otherwise.
        """
        pass

    def propagate_consumed(
        self,
        value: bool,
        props: Set[str],
        propagated: Optional[Set['ISignalInstance']]=None
    ) -> None:
        """
        Called either by a child, a parent or a propagate
        :param value:
        :param props:
        :param propagated:
        """
        # Make sure not to cause infinite recursion through the causal group
        if propagated is None:
            propagated = set()
        elif self in propagated:
            return

        self._unwritten_props -= props
        for prop in props:
            if prop in self._suitors_per_property:
                # Make sure that suitors no longer reference this signal instance
                for suitor in self._suitors_per_property[prop]:
                    suitor.wiped(self)
                del self._suitors_per_property[prop]

    def wiped(self, child: 'ISignalInstance') -> None:
        pass

    def wipe(self) -> None:
        pass

    def tick(self) -> None:
        pass

    def age(self) -> int:
        pass

    def stale(self) -> bool:
        pass
