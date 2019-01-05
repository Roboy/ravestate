# Ravestate context class

from threading import Thread, Lock, Event
from typing import Optional, Any, Tuple, Set, Dict, Generator
from collections import defaultdict

from ravestate.icontext import IContext
from ravestate.module import Module
from ravestate.state import State
from ravestate.property import PropertyBase
from ravestate.iactivation import IActivation
from ravestate.activation import Activation
from ravestate import registry
from ravestate import argparse
from ravestate.config import Configuration
from ravestate.constraint import s, Signal
from ravestate.spike import Spike

from reggol import get_logger
logger = get_logger(__name__)


class Context(IContext):

    default_signal_names: Tuple[str] = (":startup", ":shutdown", ":idle")
    default_property_signal_names: Tuple[str] = (":changed", ":pushed", ":popped", ":deleted")

    core_module_name = "core"
    import_modules_config = "import"
    tick_rate_config = "tickrate"

    _lock: Lock

    _properties: Dict[str, PropertyBase]
    _states: Set[State]
    _spikes: Set[Spike]
    _act_per_state_per_signal_age: Dict[
        Signal,
        Dict[
            int,
            Dict[
                State,
                Set[Activation]
            ]
        ]
    ]

    # This data structure is used to complete state constraints:
    #  If a state depends on a signal X that is an effect of signals
    #  Y or Z, the constraint will become (Y & X) | (Z & X).
    _signal_causes: Dict[Signal, Set[Signal]]

    _config: Configuration
    _core_config: Dict[str, Any]
    _run_task: Thread
    _shutdown_flag: Event

    def __init__(self, *arguments):
        """
        Construct a context from command line arguments.
        :param arguments: A series of command line arguments which can be parsed
         by the ravestate command line parser (see argparse.py).
        """
        modules, overrides, config_files = argparse.handle_args(*arguments)
        self._config = Configuration(config_files)
        self._core_config = {
            self.import_modules_config: [],
            self.tick_rate_config: 20
        }
        self._config.add_conf(Module(name=self.core_module_name, config=self._core_config))
        self._lock = Lock()
        self._shutdown_flag = Event()
        self._properties = dict()
        self._states = set()
        self._spikes = set()
        self._act_per_state_per_signal_age = dict()
        self._signal_causes = dict()
        self._run_task = None

        # Register default signals
        for signame in self.default_signal_names:
            self._add_sig(s(signame))

        # Set required config overrides
        for module_name, key, value in overrides:
            self._config.set(module_name, key, value)

        # Load required modules
        for module_name in self._core_config[self.import_modules_config]+modules:
            self.add_module(module_name)

    def emit(self, signal: Signal, parents: Set[Spike]=None) -> None:
        """
        Emit a signal to the signal processing loop, by creating
         a new Spike for the specified Signal.
        Note: The signal will only be processed if run() has been called!
        :param signal: The signal to be emitted.
        :param parents: The signal's parents, if it is supposed to be integrated into a causal group.
        """
        with self._lock:
            self._spikes.add(
                Spike(sig=signal.name, parents=parents, properties=set(self._properties.keys())))

    def run(self) -> None:
        """
        Creates a signal processing thread, starts it, and emits the :startup signal.
        """
        if self._run_task:
            logger.error("Attempt to start context twice!")
            return
        self._run_task = Thread(target=self._run_private)
        self._run_task.start()
        self.emit(s(":startup"))

    def shutting_down(self) -> bool:
        """
        Retrieve the shutdown flag value, which indicates whether shutdown() has been called.
        """
        return self._shutdown_flag.is_set()

    def shutdown(self) -> None:
        """
        Sets the shutdown flag and waits for the signal processing thread to join.
        """
        self._shutdown_flag.set()
        self.emit(s(":shutdown"))
        self._run_task.join()

    def add_module(self, module_name: str) -> None:
        """
        Add a module by python module folder name, or by ravestate module name.
        :param module_name: The name of the module to be added. If it is the
         name of a python module that has not been imported yet, the python module
         will be imported, and any ravestate modules registered during the python
         import will also be added to this context.
        """
        if registry.has_module(module_name):
            self._module_registration_callback(registry.get_module(module_name))
            return
        registry.import_module(module_name=module_name, callback=self._module_registration_callback)

    def add_state(self, *, st: State) -> None:
        """
        Add a state to this context. It will be indexed wrt/ the properties/signals
         it depends on. Error messages will be generated for unknown signals/properties.
        :param st: The state which should be added to this context.
        """
        if st in self._states:
            logger.error(f"Attempt to add state `{st.name}` twice!")
            return

        # make sure that all of the state's depended-upon properties exist
        for prop in st.read_props+st.write_props:
            if prop not in self._properties:
                logger.error(f"Attempt to add state which depends on unknown property `{prop}`!")
                return

        # register the state's signal
        with self._lock:
            if st.signal():
                self._add_sig(st.signal())

            # add state's constraints as causes for the written prop's :changed signals,
            #  as well as the state's own signal.
            for sig in st.constraint.signals():
                for propname in st.write_props:
                    self._signal_causes[s(f"{propname}:changed")].add(sig)
                if st.signal():
                    self._signal_causes[st.signal()].add(sig)

            # check to recognize states using old signal implementation
            if isinstance(st.constraint, str):
                logger.error(f"Attempt to add state which depends on a signal `{st.constraint}`  "
                             f"defined as a String and not Signal.")
                return

            # make sure that all of the state's depended-upon signals exist,
            #  add a default state activation for every constraint.
            # TODO: Constraint completion: If a state is constrained on a child signal C
            #  that is dependent on parent signal A or B, the state's constraint
            #  must be expanded as follows: C -> (A & C) | (B & C)
            #  This will allow indirect activations to be anticipated early.
            self._new_state_activation(st)
            self._states.add(st)

    def rm_state(self, *, st: State) -> None:
        """
        Remove a state from this context. Note, that any state which is constrained
         on the signal that is emitted by the deleted state will also be deleted.
        :param st: The state to remove. An error message will be generated,
         if the state was not previously added to this context with add_state().
        """
        if st not in self._states:
            logger.error(f"Attempt to remove unknown state `{st.name}`!")
            return
        with self._lock:
            # Remove the state's signal
            if st.signal:
                self._rm_sig(st.signal())
            # Remove state activations for the state
            self._del_state_activations(st)
            # Actually forget about the state
            self._states.remove(st)

    def add_prop(self, *, prop: PropertyBase) -> None:
        """
        Add a property to this context. An error message will be generated, if a property with
         the same name has already been added previously.
        :param prop: The property object that should be added.
        """
        if prop.fullname() in self._properties:
            logger.error(f"Attempt to add property {prop.fullname()} twice!")
            return
        # register property
        self._properties[prop.fullname()] = prop
        # register all of the property's signals
        with self._lock:
            for signalname in self.default_property_signal_names:
                self._add_sig(s(prop.fullname() + signalname))

    def rm_prop(self, *, prop: PropertyBase) -> None:
        """
        Remove a property from this context.
        Generates error message, if the property was not added with add_prop() to the context previously
        :param prop: The property to remove.object
        """
        if prop.fullname() not in self._properties:
            logger.error(f"Attempt to remove unknown property {prop.fullname()}!")
            return
        # remove property from context
        self._properties.pop(prop.fullname())
        states_to_remove: Set[State] = set()
        with self._lock:
            # remove all of the property's signals
            for signame in self.default_property_signal_names:
                self._rm_sig(s(prop.fullname() + signame))
            # remove all states that depend upon property
            for st in self._states:
                if prop.fullname() in st.read_props + st.write_props:
                    states_to_remove.add(st)
        for st in states_to_remove:
            self.rm_state(st=st)

    def __getitem__(self, key: str) -> Optional[PropertyBase]:
        """
        Retrieve a property object by name, that was previously added through add_prop()
         by it's full name. The full name is always the combination of the property's
         name and it's parent's name, joined with a colon: For example, if the name
         of a property is `foo` and it belongs to the module `bar` it's full name
         will be `bar:foo`.
        An error message will be generated if no property with the given name was
         added to the context, and None will be returned.
        :param key: The full name of the property.
        :return: The property object, or None, if no property with the given name
         was added to the context.
        """
        if key not in self._properties:
            logger.error(f"Attempt to retrieve unknown property by key `{key}`!")
            return None
        return self._properties[key]

    def conf(self, *, mod: str, key: Optional[str]=None) -> Any:
        """
        Get a single config value, or all config values for a particular module.
        :param mod: The module whose configuration should be retrieved.
        :param key: A specific config key of the given module, if only a single
         config value should be retrieved.
        :return: The value of a single config entry if key and module are both
         specified and valid, or a dictionary of config entries if only the
         module name is specified (and valid).
        """
        if key:
            return self._config.get(mod, key)
        return self._config.get_conf(mod)

    def lowest_upper_bound_eta(self, signals: Set[Signal]) -> int:
        """
        Called by activation when it is pressured to resign. The activation wants
         to know the earliest ETA of one of it's remaining required constraints.
        :param signals: The signals, whose ETA will be calculated, and among the
         results the minimum ETA will be returned.
        :return: Number of ticks it should take for at least one of the required
         signals to arrive. Fixed value (1) for now.
        """
        # TODO: Proper implementation w/ state runtime_upper_bound and property changed-wait
        return 1

    def signal_specificity(self, sig: Signal) -> float:
        """
        Called by state activation to determine it's constraint's specificity.
        :param sig: The signal whose specificity should be returned.
        :return: The given signal's specificity.
        """
        # Count activations which are interested in the signal
        if sig not in self._act_per_state_per_signal_age:
            return .0
        num_suitors = sum(
            1
            for acts_per_state in self._act_per_state_per_signal_age[sig].values()
            for acts in acts_per_state.values())
        if num_suitors > 0:
            return 1./num_suitors
        else:
            return .0

    def reacquire(self, act: IActivation, sig: Signal):
        """
        Called by activation, to indicate, that it needs a new Spike
         for the specified signal, and should for this purpose be referenced by context.
        Note: Not thread-safe, sync must be guaranteed by caller.
        :param act: The activation that needs a new spike of the specified nature.
        :param sig: Signal type for which a new spike is needed.
        """
        assert isinstance(act, Activation)  # No way around it to avoid import loop
        if sig not in self._act_per_state_per_signal_age:
            logger.error(f"Attempt to reacquire for unknown signal {sig.name}!")
            return
        interested_acts = self._act_per_state_per_signal_age[sig][0][act.state_to_activate]  # sig.min_age
        interested_acts.add(act)

    def withdraw(self, act: IActivation, sig: Signal):
        """
        Called by activation to make sure that it isn't referenced
         anymore as looking for the specified signal.
        This might be, because the activation chose to eliminate itself
         due to activation pressure, or because one of the activations
         conjunctions was fulfilled, so it is no longer looking for
         signals to fulfill the remaining conjunctions.
        Note: Not thread-safe, sync must be guaranteed by caller.
        :param act: The activation that has lost interest in the specified signal.
        :param sig: Signal type for which interest is lost.
        """
        assert isinstance(act, Activation)  # No way around it to avoid import loop
        if sig not in self._act_per_state_per_signal_age:
            logger.error(f"Attempt to withdraw for unknown signal {sig.name}!")
            return
        interested_acts = self._act_per_state_per_signal_age[sig][0][act.state_to_activate]  # sig.min_age
        interested_acts.discard(act)

    def _add_sig(self, sig: Signal):
        if sig in self._act_per_state_per_signal_age:
            logger.error(f"Attempt to add signal f{sig.name} twice!")
            return
        self._signal_causes[sig] = set()
        self._act_per_state_per_signal_age[sig] = defaultdict(lambda: defaultdict(set))

    def _rm_sig(self, sig: Signal) -> None:
        affected_states: Set[State] = set()
        for _, acts_per_state in self._act_per_state_per_signal_age[sig].items():
            affected_states |= set(acts_per_state.keys())
        logger.warning(
            f"Since signal {sig.name} was removed, the following states will have dangling constraints: " +
            ",".join(st.name for st in affected_states))
        # Remove signal as a cause for other signals
        for _, causes in self._signal_causes.items():
            causes.discard(sig)
        del self._signal_causes[sig]
        self._act_per_state_per_signal_age.pop(sig)

    def _module_registration_callback(self, mod: Module):
        self._config.add_conf(mod)
        for prop in mod.props:
            self.add_prop(prop=prop)
        for st in mod.states:
            self.add_state(st=st)
        logger.info(f"Module {mod.name} added to session.")

    def _new_state_activation(self, st: State) -> None:
        activation = Activation(st, self)
        for signal in st.constraint.signals():
            if signal in self._act_per_state_per_signal_age:
                # TODO: (Here and below) Determine, whether indexing by min age is actually necessary
                #  -> may not, because immediate acquisition is necessary -> otherwise, the
                #   signal's causal group will not know about a possibly higher-
                #   specificity state that runs a bit later.
                # self._act_per_state_per_signal_age[signal][signal.min_age][st] |= {activation}
                self._act_per_state_per_signal_age[signal][0][st] |= {activation}
            else:
                logger.error(
                    f"Adding state activation for f{st.name} which depends on unknown signal `{signal}`!")

    def _del_state_activations(self, st: State) -> None:
        activations_to_wipe: Set[Activation] = set()
        for signal in st.constraint.signals():
            if signal in self._act_per_state_per_signal_age:
                for act in self._act_per_state_per_signal_age[signal][0][st]:  # signal.min_age
                    activations_to_wipe.add(act)
                del self._act_per_state_per_signal_age[signal][0][st]  # signal.min_age
        for act in activations_to_wipe:
            act.dereference(spike=None, reacquire=False, reject=True)

    def _state_activations(self) -> Set[Activation]:
        return set(
            act
            for acts_per_state_per_age in self._act_per_state_per_signal_age.values()
            for acts_per_state in acts_per_state_per_age.values()
            for acts in acts_per_state.values()
            for act in acts)

    def _run_private(self):

        if self._core_config[self.tick_rate_config] < 1:
            logger.error("Attempt to set core config `tickrate` to a value less-than 1!")
            self._core_config[self.tick_rate_config] = 1

        tick_interval = 1. / self._core_config[self.tick_rate_config]
        while not self._shutdown_flag.wait(tick_interval):

            with self._lock:
                # Some acts may be removed during acquisition, but still need to be updated to run later.
                all_activations = self._state_activations()

                # Acquire new state activations for every spike
                for spike in self._spikes:
                    for state, acts in self._act_per_state_per_signal_age[s(spike.name())][spike.age()].items():
                        old_acts = acts.copy()
                        for act in old_acts:
                            if act.acquire(spike):
                                # Remove the Activation instance from _act_per_state_per_signal_age
                                #  for the Spike with a certain minimum age. In place of the removed
                                #  activation, if no activation with the same target state is left,
                                #  a new Activation will be created.
                                acts.remove(act)
                                if len(acts) == 0:
                                    self._new_state_activation(state)
                            else:
                                logger.error(
                                    "An activation rejected a spike it was registered to be interested in.")

                # Update all state activations.
                all_activations |= self._state_activations()
                for act in all_activations:
                    act.update()

                # Forget fully unreferenced spikes
                old_spike_set = self._spikes.copy()
                for spike in old_spike_set:
                    with spike.causal_group() as cg:
                        if cg.stale(spike):
                            # This should lead to the deletion of the spike
                            self._spikes.remove(spike)
                            spike.wipe(already_wiped_in_causal_group=True)
                            logger.debug(f"{cg}.stale({spike.name()}) -> 1")

                # Increment age on active spikes
                for spike in self._spikes:
                    spike.tick()
