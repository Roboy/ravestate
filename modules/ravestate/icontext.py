# Interface for the context towards activation and
#  context/property wrappers (needed to preempt cyclic dependencies).


from ravestate import property
from ravestate import state
from typing import Set, Any
from ravestate.constraint import Signal
from ravestate.spike import Spike
from ravestate.iactivation import IActivation


class IContext:

    def __getitem__(self, key):
        """
        Retrieve a property object by name, that was previously added through add_prop()
         by it's full name. The full name is always the combination of the property's
         name and it's parent's name, joined with a colon: For example, if the name
         of a property is `foo` and it belongs to the module `bar` it's full name
         will be `bar:foo`.
        An error message will be generated if no property with the given name was
         added to the context, and None will be returned.

        * `key`: The full name of the property.

        **Returns:** The property object, or None, if no property with the given name
         was added to the context.
        """
        pass

    def add_prop(self, *, prop: property.PropertyBase):
        """
        Add a property to this context. An error message will be generated,
         if a property with the same name has already been added previously.

        * `prop`: The property object that should be added.
        """
        pass

    def rm_prop(self, *, prop: property.PropertyBase):
        """
        Remove a property from this context.
        Generates error message, if the property was not added with add_prop()
         to the context previously.

        * `prop`: The property to remove.object
        """
        pass

    def emit(self, signal: Signal, parents: Set[Spike] = None, wipe: bool = False, payload: Any = None) -> None:
        """
        Emit a signal to the signal processing loop. _Note:_
         The signal will only be processed if #run() has been called!

        * `signal`: The signal to be emitted.

        * `parents`: The signal's parents, if it is supposed to be integrated into a causal group.

        * `wipe`: Boolean to control, whether #wipe(signal) should be called
         before the new spike is created.

        * `payload`: Value that should be embedded in the new spike.
        """
        pass

    def wipe(self, signal: Signal):
        """
        Delete all spikes for the given signal. Partially fulfilled states
         that have acquired an affected spike will be forced to reject it.
        Wiping a parent spike will also wipe all child spikes.

        * `signal`: The signal for which all existing spikes (and their children)
         should be invalidated and forgotten.
        """
        pass

    def add_state(self, *, st: state.State):
        """
        Add a state to this context. It will be indexed wrt/ the properties/signals
         it depends on. Error messages will be generated for unknown signals/properties.

        * `st`: The state which should be added to this context.
        """
        pass

    def rm_state(self, *, st: state.State):
        """
        Remove a state from this context. Note, that any state which is constrained
         on the signal that is emitted by the deleted state will also be deleted.

        * `st`: The state to remove. An error message will be generated,
         if the state was not previously added to this context with add_state().
        """
        pass

    def shutting_down(self):
        """
        Retrieve the shutdown flag value, which indicates whether shutdown() has been called.
        """
        pass

    def shutdown(self):
        """
        Sets the shutdown flag and waits for the signal processing thread to join.
        """
        pass

    def conf(self, *, mod, key=None):
        """
        Get a single config value, or all config values for a particular module.

        * `mod`: The module whose configuration should be retrieved.

        * `key`: A specific config key of the given module, if only a single
         config value should be retrieved.

        **Returns:** The value of a single config entry if key and module are both
         specified and valid, or a dictionary of config entries if only the
         module name is specified (and valid).
        """
        pass

    def lowest_upper_bound_eta(self, signals: Set[Signal]) -> int:
        """
        Called by activation when it is pressured to resign. The activation wants
         to know the earliest ETA of one of it's remaining required constraints.

        * `signals`: The signals, whose ETA will be calculated, and among the
         results the minimum ETA will be returned.

        **Returns:** Number of ticks it should take for at least one of the required
         signals to arrive. Fixed value (1) for now.
        """
        pass

    def signal_specificity(self, sig: Signal) -> float:
        """
        Called by state activation to determine it's constraint's specificity.

        * `sig`: The signal whose specificity should be returned.

        **Returns:** The given signal's specificity.
        """
        pass

    def reacquire(self, act: IActivation, sig: Signal):
        """
        Called by activation to go shopping for a new Spike
         for the specified signal, and should for this purpose be referenced by context.

        * `act`: The activation that needs a new spike of the specified nature.

        * `sig`: Signal type for which a new spike is needed.
        """
        pass

    def withdraw(self, act: IActivation, sig: Signal):
        """
        Called by activation to make sure that it isn't referenced
         anymore as looking for the specified signal.
        This might be, because the activation chose to eliminate itself
         due to activation pressure, or because one of the activations
         conjunctions was fulfilled, so it is no longer looking for
         signals to fulfill the remaining conjunctions.

        * `act`: The activation that has lost interest in the specified signal.

        * `sig`: Signal type for which interest is lost.
        """
        pass

    def secs_to_ticks(self, seconds: float) -> int:
        """
        Convert seconds to an equivalent integer number of ticks,
         given this context's tick rate.

        * `seconds`: Seconds to convert to ticks.

        **Returns:** An integer tick count.
        """
        pass
