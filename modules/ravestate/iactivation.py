# Interface of state activation towards signal instance
# Signal instance interface towards state activation

from typing import Set


class ISignalInstance:
    """
    Base interface class for signal instances.
    """

    def name(self) -> str:
        """
        Returns the name if this signal instance's signal.
        """
        pass


class IActivation:
    """
    Base interface class for state activations.
    """

    name: str

    def write_props(self) -> Set[str]:
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
        """
        pass

    def signal_instances(self) -> Set[ISignalInstance]:
        """
        Called by causal group, to remove this activation's signal instances
         from it's internal activation candidate index once they are promised
         to this activation.
        """
        pass
