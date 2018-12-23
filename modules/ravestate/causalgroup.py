# Ravestate class which encapsulates a graph of signal parent/offspring instances

from typing import Set, Dict, Optional
from ravestate.iactivation import IActivation, ISignalInstance
from threading import Lock
from collections import defaultdict

from reggol import get_logger
logger = get_logger(__name__)


class CausalGroup:
    """
    Class which represents a causal group graph of signal parent/offspring
     signal instances (a "supersignal"). These must synchronize wrt/ their (un)written
     properties and state activation candidates, such that they don't cause output races.

    Note: Always use a `with ...` construct to interact with a causal group.
     Otherwise, undefined behavior may occur due to race conditions.
    """

    # Lock for the whole causal group
    _lock: Lock
    # Remember the locked lock, since the lock member might be retargeted in merge()
    _locked_lock: Optional[Lock]

    # Set of property names for which no writing state has been
    #  activated yet within this causal group.
    #
    # This is different from _refc_index:
    #
    #  -> _unwritten_props might lack properties that are only
    #   *temporarily* unavailable, because a writing state is still running,
    #   and might still resign, which would make that state's write prop's
    #   available again.
    #
    #  -> In this case it is useful to still see the other
    #   state activation candidates for those props in _suitors_per_property.
    #
    # Also, _refc_index needs to be as
    #  sparse as possible to optimise performance.
    #
    _unwritten_props: Set[str]

    # Refcount per state activations per signal instance per property name.
    #  Refcount, because one activation may hold multiple references to one signal for multiple
    #  differently timed constraints. We essentially keep a {(propname, activation, signal, refcount)}
    #  index structure, that will be necessary for the following use-cases:
    #
    #  * Determining whether an activation is the most specific for a certain property set
    #    -> Gather state activations per instance (i) per property (p), sort by specificity
    #    -> O(i * log p)
    #  * Determining whether a signal is stale (no more activation references)
    #    -> For every property name (p), check whether the signal (i) is registered
    #    -> O(p * log i)
    #
    #  Container operations:
    #
    #  * Inserting a signal instance (i) / state activation (a)
    #    -> O(log p + log i + log a)
    #  * Removing a state activation (a) for a signal instance (i) (for each of the state's referenced properties (p))
    #    -> O(log p + log i + log a)
    #  * Wiping a signal instance (i)
    #    -> O(p * log i)
    #
    _ref_index: Dict[
        str,  # Property name
        Dict[
            ISignalInstance,
            Dict[
                IActivation,
                int
            ]
        ]
    ]

    def __init__(self, properties: Set[str]):
        """
        Create a new causal group, with a set of unwritten props.
        """
        self._lock = Lock()
        self._unwritten_props = properties.copy()
        self._ref_index = {
            # TODO: Create refc prop entries on demand
            prop: defaultdict(lambda: defaultdict(int))
            for prop in properties
        }

    def __enter__(self) -> 'CausalGroup':
        # Remember current lock, since it might change,
        # if the lock object is switched in merge()
        lock = self._lock
        self._lock.__enter__()
        if lock != self._lock:
            lock.__exit__(None, None, None)
            return self.__enter__()
        # Remember the locked lock, since the lock member might be retargeted in merge()
        self._locked_lock = self._lock
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        if self._locked_lock:
            result = self._locked_lock.__exit__(exc_type, exc_value, traceback)
            self._locked_lock = None
            return result
        logger.error(f"Exit called before enter causal group!")
        return True

    def __eq__(self, other) -> bool:
        return isinstance(other, CausalGroup) and other._lock == self._lock

    def __hash__(self):
        return self._lock.__hash__()

    def merge(self, other: 'CausalGroup'):
        """
        Merge this causal group with another. Unwritten props will become
         the set intersection of this group's unwritten props and
         other's unwritten props. consumed() will be called with
         all properties that are consumed by other, but not this.
        Afterwards, other's member objects will be set to this's.
        """
        self._unwritten_props = self._unwritten_props & other._unwritten_props
        allowed_propnames = self._ref_index.keys() & other._ref_index.keys()
        self.consumed(set(self._ref_index.keys()) - allowed_propnames)
        other.consumed(set(other._ref_index.keys()) - allowed_propnames)
        # Retarget other's members
        other._lock = self._lock
        other._unwritten_props = self._unwritten_props
        other._ref_index = self._ref_index

    def acquired(self, sig: 'ISignalInstance', acquired_by: IActivation) -> bool:
        """
        Called by Activation to notify the signal instance, that
         it is being referenced by an activation constraint.
        :param acquired_by: State activation instance,
         which is interested in this property.
        :return: Returns True if all of the acquiring's write-props are
         free, and the group now refs. the activation, False otherwise.
        """
        # Make sure that all properties are actually still writable
        for prop in acquired_by.write_props():
            if prop not in self._ref_index:
                return False
        for prop in acquired_by.write_props():
            self._ref_index[prop][sig][acquired_by] += 1
        return True

    def rejected(self, sig: 'ISignalInstance', rejected_by: IActivation) -> None:
        """
        Called by a state activation, to notify the group that a member signal-
         instance is no longer being referenced for the given state's write props.
        This may be either because the state activation resigned,
         or because this signal instance got too old.
        :param sig: The member signal whose ref-set should be reduced.
        :param rejected_by: State activation instance, which is no longer
         interested in this property.
        """
        for prop in rejected_by.write_props():
            if prop in self._ref_index and \
               sig in self._ref_index[prop] and \
               rejected_by in self._ref_index[prop][sig]:
                remaining_refc = self._ref_index[prop][sig][rejected_by]
                if remaining_refc > 0:
                    self._ref_index[prop][sig][rejected_by] -= 1
                    if remaining_refc > 1:
                        # do not fall through to ref deletion
                        continue
                else:
                    logger.error(f"Attempt to deref group for unref'd activation {rejected_by.name}")

                del self._ref_index[prop][sig][rejected_by]
                if len(self._ref_index[prop][sig]) == 0:
                    del self._ref_index[prop][sig]

    def pressure_activation(self, ready_suitor: IActivation) -> bool:
        """
        Called by constraint, to inquire whether this causal group would happily
         be consumed for the given state activation's properties.
        This will be called periodically on the group by state activations
         that are ready to go. Therefore, a False return value from this
         function is never a final judgement (more like a "maybe later").
        As soon as consumption is consented though, the given state activation
         will be removed from this signals referenced activations!
        :param ready_suitor: The state activation which would like to consume
         this instance for it's write props.
        :return: True if this instance agrees to proceeding with the given consumer
         for the consumer's write props, False otherwise.
        """
        # TODO: Gather candidate state activations that compete with ready_suitor
        #  For now, just abort once the first higher-specificity candidate is found.
        specificity = ready_suitor.specificity()
        for prop in ready_suitor.write_props():
            if prop in self._unwritten_props:
                for sig in self._ref_index[prop]:
                    for candidate in self._ref_index[prop][sig]:
                        # TODO: Pressure higher specificity candidates to make a decision
                        if candidate.specificity() > specificity:
                            return False
            else:
                # Easy exit condition: prop not free for writing
                return False
        # Remove the consented activation from the ref index
        for sig in ready_suitor.signal_instances():
            self.rejected(sig, ready_suitor)
        # Mark the consented w-props as unavailable
        self._unwritten_props -= ready_suitor.write_props()
        return True

    def consumed(self, consumed_props: Set[str]) -> None:
        """
        Called by activation to notify the signal, that it has been
         consumed for the given set of properties.
        :param consumed_props: The properties which have been consumed.
        """
        for prop in consumed_props:
            # Notify all concerned activations, that the
            # signals they are referencing are no longer available
            for sig in self._ref_index[prop]:
                for act in self._ref_index[prop][sig]:
                    act.wiped(sig)
            # Remove the consumed prop from the index
            del self._ref_index[prop]

    def wiped(self, sig: 'ISignalInstance') -> None:
        """
        Called by a signal instance, to notify the causal group that
         it was wiped and should no longer be remembered.
        :param sig: The instance that should be henceforth forgotten.
        """
        for prop in self._ref_index:
            if sig in self._ref_index[prop]:
                for act in self._ref_index[prop][sig]:
                    act.wiped(sig)
                del self._ref_index[prop][sig]

    def stale(self, sig: 'ISignalInstance') -> bool:
        """
        Determine, whether a signal instance is stale (has no
        remaining interested activations and no children).
        :return: True, if no activations reference the given
         signal for any unwritten property. False otherwise.
        """
        for prop in self._ref_index:
            if sig in self._ref_index[prop]:
                if len(self._ref_index[prop][sig]) > 0:
                    return False
                else:
                    # Do some cleanup
                    del self._ref_index[prop][sig]
        return True


