# Ravestate context class
from threading import Thread, RLock, Event
from typing import Optional, Any, Tuple, Set, Dict, Iterable, List, Generator
from collections import defaultdict
from math import ceil
from copy import deepcopy
import gc

from ravestate.wrappers import PropertyWrapper

from ravestate.icontext import IContext
from ravestate.module import Module, has_module, get_module, import_module
from ravestate.state import State
from ravestate.property import Property
from ravestate.iactivation import IActivation
from ravestate.activation import Activation
from ravestate import argparser
from ravestate.config import Configuration
from ravestate.constraint import *
from ravestate.spike import Spike

from reggol import get_logger
logger = get_logger(__name__)


CORE_MODULE_NAME = "core"
IMPORT_MODULES_CONFIG_KEY = "import"
TICK_RATE_CONFIG_KEY = "tickrate"
CORE_MODULE_CONFIG = {
    IMPORT_MODULES_CONFIG_KEY: [],
    TICK_RATE_CONFIG_KEY: 20
}

with Module(name="core", config=CORE_MODULE_CONFIG):

    """
    The startup signal, which is fired once when `Context.run()` is executed.<br>
    __Hint:__ All key-word arguments of #constraint.Signal(...)
     (`min_age`, `max_age`, `detached`) are supported.
    """
    sig_startup = Signal("startup")

    """
    Obtain the shutdown signal, which is fired once when `Context.shutdown()` is called.<br>
    __Hint:__ All key-word arguments of #constraint.Signal(...)
     (`min_age`, `max_age`, `detached`) are supported.
    """
    sig_shutdown = Signal("shutdown")

    prop_pressure = Property(
        name="pressure",
        allow_read=True,
        allow_write=True,
        allow_push=False,
        allow_pop=False,
        default_value=False,
        always_signal_changed=False,
        is_flag_property=True)

    prop_activity = Property(
        name="activity",
        allow_read=True,
        allow_write=True,
        allow_push=False,
        allow_pop=False,
        default_value=0,
        always_signal_changed=False)


def create_and_run_context(*args, runtime_overrides=None):
    """
    Creates a new Context with the given parameters and runs it
    """
    context = Context(*args, runtime_overrides=runtime_overrides)
    context.run()


class _Grab:
    """
    Helper class to grab an equivalent set item, copied from here:<br>
      http://python.6.x6.nabble.com/Get-item-from-set-td1530758.html

    ```python
    grab = _Grab("mysig")
    if grab in myset:
        print(grab.result)
    ```
    """
    def __init__(self, value):
        self.search_value = value
        self.result = None

    def __hash__(self):
        return hash(self.search_value)

    def __eq__(self, other):
        if self.search_value == other:
            self.result = other
            return True
        return False


class Context(IContext):
    _default_signals: Tuple[Signal] = (sig_startup, sig_shutdown)
    _default_properties: Tuple[Property] = (prop_activity, prop_pressure)

    _lock: RLock

    _properties: Dict[str, Property]
    _spikes_per_signal: Dict[
        Signal,
        Set[Spike]
    ]

    # Some activations that have all constraints fulfilled
    #  still need to be updated, because they are waiting for
    #  their turn. While they wait, they are stored here.
    # Also, this is just a pretty fucking useful index.
    _activations_per_state: Dict[State, Set[Activation]]

    # This is the bar part of the gaybar - here, activations
    #  register for certain signal spikes which they still need to fulfill.
    _needy_acts_per_state_per_signal: Dict[
        Signal,
        Dict[
            State,
            Set[Activation]
        ]
    ]

    # This data structure is used to complete state constraints:
    #  If a state depends on a signal X that is an effect of signals
    #  Y or Z, the constraint will become (Y & X) | (Z & X).
    _signal_causes: Dict[Signal, List[Conjunct]]

    _config: Configuration
    _core_config: Dict[str, Any]
    _run_task: Optional[Thread]
    _shutdown_flag: Event

    def __init__(self, *arguments, runtime_overrides: List[Tuple[str, str, Any]] = None):
        """
        Construct a context from command line arguments.

        * `arguments`: A series of command line arguments which can be parsed
         by the ravestate command line parser (see argparser.py).
        * `runtime_overrides`: A list of config overrides in the form of (modulename, key, value).
         Can be used to set config entries to values other than strings or lists like in command line arguments.
         An example use-case is a module that starts a new context (in separate process) and can set
         config entries to Connection Objects to enable communication between old and new context.
        """
        modules, overrides, config_files = argparser.handle_args(*arguments)
        self._config = Configuration(config_files)
        self._lock = RLock()
        self._shutdown_flag = Event()
        self._properties = dict()
        self._spikes_per_signal = defaultdict(set)
        self._needy_acts_per_state_per_signal = dict()
        self._signal_causes = dict()
        self._activations_per_state = dict()
        self._run_task = None

        # Load required modules
        self.add_module(CORE_MODULE_NAME)
        for module_name in self.conf(mod=CORE_MODULE_NAME, key=IMPORT_MODULES_CONFIG_KEY) + modules:
            self.add_module(module_name)

        # Set required config overrides
        for module_name, key, value in overrides:
            self._config.set(module_name, key, value)
        # Set additional config runtime overrides
        if runtime_overrides:
            for module_name, key, value in runtime_overrides:
                self._config.set(module_name, key, value)

        self.tick_rate = self.conf(mod=CORE_MODULE_NAME, key=TICK_RATE_CONFIG_KEY)
        if self.tick_rate < 1:
            logger.error("Attempt to set core config `tickrate` to a value less-than 1!")
            self.tick_rate = 1

    def emit(self, signal: Signal, parents: Set[Spike]=None, wipe: bool=False, payload: Any=None) -> None:
        """
        Emit a signal to the signal processing loop. _Note:_
         The signal will only be processed if #run() has been called!

        * `signal`: The signal to be emitted.

        * `parents`: The signal's parents, if it is supposed to be integrated into a causal group.

        * `wipe`: Boolean to control, whether #wipe(signal) should be called
         before the new spike is created.

        * `payload`: Value that should be embedded in the new spike.
        """
        if wipe:
            self.wipe(signal)
        with self._lock:
            new_spike = Spike(
                sig=signal.id(),
                parents=parents,
                consumable_resources=set(self._properties.keys()),
                payload=payload)
            logger.debug(f"Emitting {new_spike}")
            self._spikes_per_signal[signal].add(new_spike)

    def wipe(self, signal: Signal):
        """
        Delete all spikes for the given signal. Partially fulfilled states
         that have acquired an affected spike will be forced to reject it.
        Wiping a parent spike will also wipe all child spikes.

        * `signal`: The signal for which all existing spikes (and their children)
         should be invalidated and forgotten.
        """
        with self._lock:
            for spikes in self._spikes_per_signal.values():
                for spike in spikes:
                    if spike.id() == signal.id():
                        spike.wipe()
        # Final cleanup will be performed while update is running,
        #  and cg.stale(spike) returns true.
        # TODO: Make sure, that it is not a problem if the spike is currently referenced
        #  in a running state that would give it new offspring (and a second life).

    def run(self) -> None:
        """
        Creates a signal processing thread, starts it, and emits the core:startup signal.
        """
        if self._run_task:
            logger.error("Attempt to start context twice!")
            return
        self._run_task = Thread(target=self._run_loop)
        self._run_task.start()
        self.emit(sig_startup)

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
        self.emit(sig_shutdown)
        self._run_task.join()

    def add_module(self, module_name: str) -> None:
        """
        Add a module by python module folder name, or by ravestate module name.

        * `module_name`: The name of the module to be added. If it is the
         name of a python module that has not been imported yet, the python module
         will be imported, and any ravestate modules registered during the python
         import will also be added to this context.
        """
        if has_module(module_name):
            self._module_registration_callback(get_module(module_name))
            return
        import_module(module_name=module_name, callback=self._module_registration_callback)

    def add_state(self, *, st: State) -> None:
        """
        Add a state to this context. It will be indexed wrt/ the properties/signals
         it depends on. Error messages will be generated for unknown signals/properties.

        * `st`: The state which should be added to this context.
        """
        if st in self._activations_per_state:
            logger.error(f"Attempt to add state `{st.name}` twice!")
            return

        # make sure that all of the state's depended-upon properties exist
        for prop_id in st.get_all_props_ids():
            if prop_id not in self._properties:
                logger.error(f"Attempt to add state which depends on unknown property `{prop_id}`!")
                return

        # replace configurable ages with their config values
        for signal in st.constraint.signals():
            if isinstance(signal.min_age_value, ConfigurableAge):
                conf_entry = self.conf(mod=st.module_name, key=signal.min_age_value.key)
                if conf_entry is None:
                    logger.error(f"Could not set min_age for cond of state {st.name} in module {st.module_name}")
                    signal.min_age_value = 0.
                else:
                    signal.min_age_value = conf_entry
            if isinstance(signal.max_age_value, ConfigurableAge):
                conf_entry = self.conf(mod=st.module_name, key=signal.max_age_value.key)
                if conf_entry is None:
                    logger.error(f"Could not set max_age for cond of state {st.name} in module {st.module_name}")
                    signal.max_age_value = 5.
                else:
                    signal.max_age_value = conf_entry

        # register the state's signal
        with self._lock:
            if st.signal:
                self._add_sig(st.signal)

            # add state's constraints as causes for the written prop's :changed signals,
            #  as well as the state's own signal.
            states_to_recomplete: Set[State] = {st}
            if not st.emit_detached:
                for conj in st.constraint.conjunctions(filter_detached=True):
                    for signal in self.possible_signals(st):
                        self._signal_causes[signal].append(conj)
                        # Since a new cause for the property's signal is added,
                        #  it must be added to all states depending on that signal.
                        states_to_recomplete.update(self._states_for_signal(signal))

            # add state to state activation map
            self._activations_per_state[st] = set()

            # make sure that all of the state's depended-upon signals exist,
            #  add a default state activation for every constraint.
            for state in states_to_recomplete:
                self._del_state_activations(state)
                self._complete_constraint(state)
                # first create an activation which reacquires for existing spikes,
                #  then create another activation which hooks into signals
                #  that are not looked for anymore by the spiky one.
                if self._new_state_activation(state, reacquire=True):
                    self._new_state_activation(state)

        # register the state's consumable dummy, so that it is passed
        #  to Spike and from there to CausalGroup as a consumable resource.
        if st.consumable.id() not in self._properties:
            self.add_prop(prop=st.consumable)

    def rm_state(self, *, st: State) -> None:
        """
        Remove a state from this context. Note, that any state which is constrained
         on the signal that is emitted by the deleted state will also be deleted.

        * `st`: The state to remove. An error message will be generated,
         if the state was not previously added to this context with add_state().
        """
        if st not in self._activations_per_state:
            logger.error(f"Attempt to remove unknown state `{st.name}`!")
            return
        with self._lock:
            # Remove the state's signal
            if st.signal:
                self._rm_sig(st.signal)
            # Remove state activations for the state
            self._del_state_activations(st)
            # Actually forget about the state
            del self._activations_per_state[st]
        # unregister the state's consumable dummy
        self.rm_prop(prop=st.consumable)

    def add_prop(self, *, prop: Property) -> None:
        """
        Add a copy of a property to this context. An error message will be generated, if a property with
         the same name has already been added previously. Note: Context will adopt a __copy__
         of the given property, the actual property will not be changed.

        * `prop`: The property object that should be added.
        """
        # prop = prop.clone()
        if prop.id() in self._properties:
            logger.error(f"Attempt to add property {prop.id()} twice!")
            return
        # Do not adopt the
        with self._lock:
            # register property
            self._properties[prop.id()] = prop
            # register all of the property's signals
            for signal in prop.signals():
                self._add_sig(signal)

    def rm_prop(self, *, prop: Property) -> None:
        """
        Remove a property from this context.
        Generates error message, if the property was not added with add_prop() to the context previously

        * `prop`: The property to remove.object
        """
        if prop.id() not in self._properties:
            logger.error(f"Attempt to remove unknown property {prop.id()}!")
            return
        # remove property from context
        self._properties.pop(prop.id())
        states_to_remove: Set[State] = set()
        with self._lock:
            # remove all of the property's signals
            for signal in prop.signals():
                self._rm_sig(signal)
            # remove all states that depend upon property
            for st in self._activations_per_state:
                if prop.id() in st.get_all_props_ids():
                    states_to_remove.add(st)
        for st in states_to_remove:
            self.rm_state(st=st)

    def __getitem__(self, key: str) -> Optional[Property]:
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
        if key not in self._properties:
            logger.error(f"Attempt to retrieve unknown property by key `{key}`!")
            return None
        return self._properties[key]

    def conf(self, *, mod: str, key: Optional[str]=None) -> Any:
        """
        Get a single config value, or all config values for a particular module.

        * `mod`: The module whose configuration should be retrieved.

        * `key`: A specific config key of the given module, if only a single
         config value should be retrieved.

        **Returns:** The value of a single config entry if key and module are both
         specified and valid, or a dictionary of config entries if only the
         module name is specified (and valid).
        """
        if key:
            return self._config.get(mod, key)
        return self._config.get_conf(mod)

    def signal_specificity(self, sig: Signal) -> float:
        """
        Called by state activation to determine it's constraint's specificity.

        * `sig`: The signal whose specificity should be returned.

        **Returns:** The given signal's specificity.
        """
        # Count activations which are interested in the signal
        if sig not in self._needy_acts_per_state_per_signal:
            return .0
        num_suitors = len(self._needy_acts_per_state_per_signal[sig])
        if num_suitors > 0:
            return 1./num_suitors
        else:
            return .0

    def reacquire(self, act: IActivation, sig: Signal):
        """
        Called by activation, to indicate, that it needs a new Spike
         for the specified signal, and should for this purpose be referenced by context.
        Note: Not thread-safe, sync must be guaranteed by caller.

        * `act`: The activation that needs a new spike of the specified nature.

        * `sig`: Signal type for which a new spike is needed.
        """
        assert isinstance(act, Activation)  # No way around it to avoid import loop
        if sig not in self._needy_acts_per_state_per_signal:
            logger.error(f"Attempt to reacquire for unknown signal {sig.id()}!")
            return
        interested_acts = self._needy_acts_per_state_per_signal[sig][act.state_to_activate]
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

        * `act`: The activation that has lost interest in the specified signal.

        * `sig`: Signal type for which interest is lost.
        """
        assert isinstance(act, Activation)  # No way around it to avoid import loop
        if sig not in self._needy_acts_per_state_per_signal:
            logger.warning(f"Attempt to withdraw for unknown signal {sig.id()}!")
            return
        interested_acts = self._needy_acts_per_state_per_signal[sig][act.state_to_activate]
        interested_acts.discard(act)

    def secs_to_ticks(self, seconds: float) -> int:
        """
        Convert seconds to an equivalent integer number of ticks,
         given this context's tick rate.

        * `seconds`: Seconds to convert to ticks.

        **Returns:** An integer tick count.
        """
        return ceil(seconds * float(self.tick_rate))

    def possible_signals(self, state: State) -> Generator[Signal, None, None]:
        """
        Yields all signals, for which spikes may be created if
         the given state is executed.

        * `state`: The state, which should be analyzed for it's
         possibly generated signals (declared signal + property-changed signals).
        """
        for prop in state.write_props:
            if isinstance(prop, str):
                if prop in self._properties:
                    prop = self._properties[prop]
                else:
                    continue  # TODO does this happen? (was there previously)
            for signal in prop.signals():  # TODO also check here if in self._properties?
                yield signal
        if state.signal:
            yield state.signal

    def run_once(self, seconds_passed=1., debug=False) -> None:
        """
        Run a single update for this context, which will ...<br>
        (0) progress cooled down state weights.<br>
        (1) reduce redundant candidate activations.<br>
        (2) associate new spikes with state activations.<br>
        (3) update state activations.<br>
        (4) forget spikes which have no suitors in their causal groups.<br>
        (5) age spikes.<br>
        (6) invoke garbage collection.<br>
        (7) update the `core:activity` and `core:pressure` variables.

        * `seconds_passed`: Seconds, as floatiing point, since the last update. Will be used
         to determine the number of ticks to add/subtract to/from spike/activation age/cooldown/deathclock.
        """
        with self._lock:

            # ---------- Update weights wrt/ cooldown for all states -----------

            for state in self._activations_per_state:
                state.update_weight(seconds_passed)

            # ----------- For every state, compress it's activations -----------

            for st, acts in self._activations_per_state.items():
                assert len(acts) > 0
                if len(acts) > 1:
                    # We could do some fancy merge of partially fulfilled activations,
                    #  but for now let's just remove all completely unfulfilled
                    #  ones apart from one.
                    allowed_unfulfilled: Optional[Activation] = None
                    for act in acts.copy():
                        if not act.spiky():
                            if allowed_unfulfilled:
                                for signal in act.constraint.signals():
                                    if signal in self._needy_acts_per_state_per_signal and \
                                            act in self._needy_acts_per_state_per_signal[signal][act.state_to_activate]:
                                        self._needy_acts_per_state_per_signal[
                                            signal][act.state_to_activate].remove(act)
                                acts.remove(act)
                            else:
                                allowed_unfulfilled = act

            # --------- Acquire new state activations for every spike ----------

            for spikes in self._spikes_per_signal.values():
                for spike in spikes:
                    if spike.is_wiped() or spike.age() > 0:
                        continue
                    for state, acts in self._needy_acts_per_state_per_signal[spike.id()].items():
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

            # ------------------ Update all state activations. -----------------

            for act in self._state_activations():
                if act.update():
                    self._state_activated(act)

            # ----------------- Forget fully unreferenced spikes ---------------

            for spikes in self._spikes_per_signal.values():
                for spike in spikes.copy():
                    with spike.causal_group() as cg:
                        if cg.stale(spike):
                            # This should lead to the deletion of the spike
                            spikes.remove(spike)
                            spike.wipe(already_wiped_in_causal_group=True)
                            logger.debug(f"{cg}.stale({spike})->Y")

            # ----------------- Increment age on active spikes -----------------

            for spikes in self._spikes_per_signal.values():
                for spike in spikes:
                    spike.tick()

            # -------------------- Force garbage collect -----------------------

            gc.collect()

        self._update_core_properties(debug=debug)

    def _state_activated(self, act: Activation):
        self._activations_per_state[act.state_to_activate].discard(act)

    def _add_sig(self, sig: Signal):
        if sig in self._needy_acts_per_state_per_signal:
            return
        self._signal_causes[sig] = []
        self._needy_acts_per_state_per_signal[sig] = defaultdict(set)

    def _rm_sig(self, sig: Signal) -> None:
        affected_states: Set[State] = set(self._needy_acts_per_state_per_signal[sig].keys())
        if affected_states:
            logger.warning(
                f"Since signal {sig.id()} was removed, the following states will have dangling constraints: " +
                ",".join(st.name for st in affected_states))
        # Remove signal as a cause for other signals
        for _, causes in self._signal_causes.items():
            old_causes = causes.copy()
            causes.clear()
            causes += [cause for cause in old_causes if sig not in cause]
        del self._signal_causes[sig]
        self._needy_acts_per_state_per_signal.pop(sig)

    def _module_registration_callback(self, mod: Module):
        self._config.add_conf(mod)
        for prop in mod.props:
            self.add_prop(prop=prop)
        for sig in mod.signals:
            self._add_sig(sig)
        for st in mod.states:
            self.add_state(st=st)
        logger.info(f"Module {mod.name} added to session.")

    def _new_state_activation(self, st: State, reacquire: bool = False) -> bool:
        activation = Activation(st, self)
        reacquired = False
        self._activations_per_state[st].add(activation)
        for signal in st.completed_constraint.signals():
            if signal in self._needy_acts_per_state_per_signal:
                signal_reacquired = False
                if reacquire:
                    for spike in self._spikes_per_signal[signal]:
                        if spike.is_wiped():
                            continue
                        if activation.acquire(spike=spike):
                            signal_reacquired = True
                            break
                if signal_reacquired:
                    reacquired = True
                else:
                    self._needy_acts_per_state_per_signal[signal][st].add(activation)
            else:
                logger.warning(
                    f"Adding state activation for {st.name} which depends on unknown signal `{signal}`!")
        return reacquired

    def _del_state_activations(self, st: State) -> None:
        # delete activation from gaybar
        if st not in self._activations_per_state:
            return
        for signal in st.completed_constraint.signals():
            if signal in self._needy_acts_per_state_per_signal:
                if st in self._needy_acts_per_state_per_signal[signal]:
                    del self._needy_acts_per_state_per_signal[signal][st]  # signal.min_age
        for act in self._activations_per_state[st].copy():
            act.dereference(spike=None, reacquire=False, reject=True)
            self._activations_per_state[st].remove(act)

    def _state_activations(self, *, st: Optional[State]=None) -> Set[Activation]:
        if st:
            return self._activations_per_state[st].copy()
        else:
            return {act for acts in self._activations_per_state.values() for act in acts}

    def _states_for_signal(self, sig: Signal) -> Iterable[State]:
        if sig not in self._needy_acts_per_state_per_signal:
            return set()
        return self._needy_acts_per_state_per_signal[sig].keys()

    def _complete_constraint(self, st: State):
        new_conjuncts: Set[Conjunct] = deepcopy(set(st.constraint.conjunctions()))
        for conj in st.constraint.conjunctions():
            known_signals = set()
            new_conjuncts.update(
                Conjunct(*conj_signals)
                for conj_signals in self._complete_conjunction(conj, known_signals))
            assert len(known_signals) == 0
        st.completed_constraint = Disjunct(*{conj for conj in new_conjuncts})

    def _complete_conjunction(self, conj: Conjunct, known_signals: Set[Signal]) -> List[Set[Signal]]:
        result = [set(deepcopy(sig) for sig in conj.signals())]

        def copy_conjunct_set_with_completion(original_conj: Set[Signal], completion_conj: Set[Signal], completed_signal: Signal):
            original_conj = deepcopy(original_conj)
            completion_conj = deepcopy(completion_conj)
            original_conj |= completion_conj
            for sig in original_conj:
                if sig == completed_signal:
                    sig.completed_by = completion_conj
                if sig in completion_conj:
                    sig.max_age_value = -1  # See #52 (§3)
                    sig.is_completion = True
            return original_conj

        for conj_sig in conj.signals():
            completion = self._complete_signal(conj_sig, known_signals)
            if completion:
                # the signal is non-cyclic, and has at least one cause (it is a secondary signal).
                #  permute existing disjunct conjunctions with new conjunction(s)
                result = [
                    copy_conjunct_set_with_completion(result_conj, completion_conj, conj_sig)
                    for result_conj in result for completion_conj in completion]

        return result

    def _complete_signal(self, sig: Signal, known_signals: Set[Signal]) -> Optional[List[Set[Signal]]]:
        # detect and handle cyclic causal chain
        if sig in known_signals:
            return None
        assert sig in self._signal_causes

        # a signal without cause (a primary signal) needs no further completion
        if not self._signal_causes[sig] or sig.detached_value:
            return None

        # a signal with at least one secondary cause needs at least one non-cyclic
        #  cause to be considered a completion itself
        result = []
        known_signals.add(sig)
        for conj in self._signal_causes[sig]:
            completion = self._complete_conjunction(conj, known_signals)
            if completion:
                result += completion
        known_signals.discard(sig)

        return result if len(result) else None

    def _update_core_properties(self, debug=False):
        with self._lock:
            pressured_acts = []
            partially_fulfilled_acts = []
            for act in self._state_activations():
                if prop_activity.changed() not in set(act.constraint.signals()):
                    if act.is_pressured():
                        pressured_acts.append(act.id)
                    if act.spiky():
                        partially_fulfilled_acts.append(act)
        PropertyWrapper(
            prop=prop_pressure,
            ctx=self,
            allow_write=True,
            allow_read=True
        ).set(len(pressured_acts) > 0)
        PropertyWrapper(
            prop=prop_activity,
            ctx=self,
            allow_write=True,
            allow_read=True
        ).set(len(partially_fulfilled_acts) > 0)
        if debug:
            partially_fulfilled_info = "; ".join(
                f"{act} -> {', '.join(repr(spike) for spike in act.spikes())}"
                for act in partially_fulfilled_acts)
            if partially_fulfilled_info:
                logger.info(partially_fulfilled_info)

    def _run_loop(self):
        tick_interval = 1. / self.tick_rate
        while not self._shutdown_flag.wait(tick_interval):
            self.run_once(tick_interval)

    def test(self) -> bool:
        """
        Execute internal integrity checks.
        """
        # Check all causal groups for refcount correctness
        checked_causal_groups = set()
        result = True
        with self._lock:
            for spikes in self._spikes_per_signal.values():
                for spike in spikes:
                    with spike.causal_group() as cg:
                        if cg not in checked_causal_groups:
                            checked_causal_groups.add(cg)
                            if not cg.check_reference_sanity():
                                result = False
        return result
