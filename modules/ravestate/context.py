# Ravestate context class

from reggol import get_logger
logger = get_logger(__name__)

from threading import Thread, Lock, Semaphore
from typing import Optional, Any, Tuple, List, Set, Dict

from ravestate import icontext
from ravestate import activation
from ravestate import module
from ravestate import state
from ravestate import property
from ravestate import registry
from ravestate import argparse
from ravestate.config import Configuration
from ravestate.constraint import s, Signal


class Context(icontext.IContext):

    default_signal_names: Tuple[str] = (":startup", ":shutdown", ":idle")
    default_property_signal_names: Tuple[str] = (":changed", ":pushed", ":popped", ":deleted")
    core_module_name = "core"
    import_modules_config = "import"

    def __init__(self, *arguments):
        """
        Construct a context from command line arguments.
        :param arguments: A series of command line arguments which can be parsed
         by the ravestate command line parser (see argparse.py).
        """
        modules, overrides, config_files = argparse.handle_args(*arguments)
        self.config = Configuration(config_files)
        self.core_config = {
            self.import_modules_config: []
        }
        self.config.add_conf(module.Module(name=self.core_module_name, config=self.core_config))

        self.signal_queue: List[Signal] = []
        self.signal_queue_lock = Lock()
        self.signal_queue_counter = Semaphore(0)
        self.run_task = None
        self.shutdown_flag = False
        self.properties = {}
        self.activation_candidates = dict()

        self.states = set()
        self.states_per_signal: Dict[Signal, Set] = {s(signal_name): set() for signal_name in self.default_signal_names}
        self.states_lock = Lock()

        # Set required config overrides
        for module_name, key, value in overrides:
            self.config.set(module_name, key, value)
        # Load required modules
        for module_name in self.core_config[self.import_modules_config]+modules:
            self.add_module(module_name)

    def emit(self, signal: Signal) -> None:
        """
        Emit a signal to the signal processing loop. Note:
         The signal will only be processed if run() has been called!
        :param signal: The signal to be emitted.
        """
        with self.signal_queue_lock:
            self.signal_queue.append(signal)
            self.signal_queue_counter.release()

    def run(self) -> None:
        """
        Creates a signal processing thread, starts it, and emits the :startup signal.
        """
        if self.run_task:
            logger.error("Attempt to start context twice!")
            return
        self.run_task = Thread(target=self._run_private)
        self.run_task.start()
        self.emit(s(":startup"))

    def shutting_down(self) -> bool:
        """
        Retrieve the shutdown flag value, which indicates whether shutdown() has been called.
        """
        return self.shutdown_flag

    def shutdown(self) -> None:
        """
        Sets the shutdown flag and waits for the signal processing thread to join.
        """
        self.shutdown_flag = True
        self.emit(s(":shutdown"))
        self.run_task.join()

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

    def add_state(self, *, st: state.State) -> None:
        """
        Add a state to this context. It will be indexed wrt/ the properties/signals
         it depends on. Error messages will be generated for unknown signals/properties.
        :param st: The state which should be added to this context.
        """
        if st in self.states:
            logger.error(f"Attempt to add state `{st.name}` twice!")
            return

        # make sure that all of the state's depended-upon properties exist
        for prop in st.read_props+st.write_props:
            if prop not in self.properties:
                logger.error(f"Attempt to add state which depends on unknown property `{prop}`!")

        # register the state's signal
        with self.states_lock:
            if st.signal:
                self.states_per_signal[st.signal] = set()
            # check to recognize states using old signal implementation
            if isinstance(st.triggers, str):
                logger.error(f"Attempt to add state which depends on a signal `{st.triggers}`  "
                              f"defined as a String and not Signal.")

            # make sure that all of the state's depended-upon signals exist
            for signal in st.triggers.get_all_signals():
                if signal in self.states_per_signal:
                    self.states_per_signal[signal].add(st)
                else:
                    logger.error(f"Attempt to add state which depends on unknown signal `{signal}`!")
            self.states.add(st)

    def rm_state(self, *, st: state.State) -> None:
        """
        Remove a state from this context.
        :param st: The state to remove. An error message will be generated,
         if the state was not previously added to this context with add_state().
        """
        if st not in self.states:
            logger.error(f"Attempt to remove unknown state `{st.name}`!")
            return
        with self.states_lock:
            if st.signal:
                self.states_per_signal.pop(st.signal)
            for signal in st.triggers.get_all_signals():
                    self.states_per_signal[signal].remove(st)
            self.states.remove(st)

    def add_prop(self, *, prop: property.PropertyBase) -> None:
        """
        Add a property to this context. An error message will be generated, if a property with
         the same name has already been added previously.
        :param prop: The property object that should be added.
        """
        if prop.fullname() in self.properties.values():
            logger.error(f"Attempt to add property {prop.name} twice!")
            return
        # register property
        self.properties[prop.fullname()] = prop
        # register all of the property's signals
        with self.states_lock:
            for signalname in self.default_property_signal_names:
                self.states_per_signal[s(prop.fullname() + signalname)] = set()

    def get_prop(self, key: str) -> Optional[property.PropertyBase]:
        """
        Retrieve a property object by that was previously added through add_prop()
         by it's full name. The full name is always the combination of the property's
         name and it's parent's name, joined with a colon: For example, if the name
         of a property is `foo` and it belongs to the module `bar` it's full name
         will be `bar:foo`.
        An error message will be generated if no property with the given name was
         added to the context, and None will be returned/
        :param key: The full name of the property.
        :return: The property object, or None, if no property with the given name
         was added to the context.
        """
        if key not in self.properties:
            logger.error(f"Attempt to retrieve unknown property by key `{key}`!")
            return None
        return self.properties[key]

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
            return self.config.get(mod, key)
        return self.config.get_conf(mod)

    def _module_registration_callback(self, mod: module.Module):
        self.config.add_conf(mod)
        for prop in mod.props:
            self.add_prop(prop=prop)
        for st in mod.states:
            self.add_state(st=st)
        logger.info(f"Module {mod.name} added to session.")

    def _run_private(self):
        while not self.shutdown_flag:
            # TODO: Recognize and signal Idleness
            self.signal_queue_counter.acquire()
            with self.signal_queue_lock:
                signal = self.signal_queue.pop(0)

            # collect states which depend on the new signal,
            # and create state activation objects for them if necessary
            logger.debug(f"Received {signal.name} ...")
            with self.states_lock:
                for state in self.states_per_signal[signal]:
                    if state.name not in self.activation_candidates:
                        self.activation_candidates[state.name] = activation.StateActivation(state, self)

            logger.debug("State activation candidates: \n"+"\n".join(
                "- "+state_name for state_name in self.activation_candidates))

            current_activation_candidates = self.activation_candidates
            self.activation_candidates = dict()
            consider_for_immediate_activation = []

            # go through candidates and remove those which want to be removed,
            # remember those which want to be remembered, forget those which want to be forgotten
            for state_name, act in current_activation_candidates.items():
                notify_return = act.notify_signal(signal)
                logger.debug(f"-> {act.state_to_activate.name} returned {notify_return} on notify_signal {signal.name}")
                if notify_return == 0:
                    self.activation_candidates[state_name] = act
                elif notify_return > 0:
                    consider_for_immediate_activation.append(act)
                # ignore act_state -1: Means that state activation is considering itself canceled

            # sort the state activations by their specificity
            consider_for_immediate_activation.sort(key=lambda act: act.specificity(), reverse=True)

            # let the state with the highest specificity claim write props first, then lower ones.
            # a state will only be activated if all of it's write-props are available.
            # TODO: Recognize same-specificity states and actively decide between them.
            claimed_write_props = set()
            for act in consider_for_immediate_activation:
                all_write_props_free = True
                for write_prop in act.state_to_activate.write_props:
                    if write_prop in claimed_write_props:
                        all_write_props_free = False
                        break
                if all_write_props_free:
                    logger.debug(f"-> Activating {act.state_to_activate.name}")
                    thread = act.run()
                    thread.start()
                else:
                    logger.debug(f"-> Dropping activation of {act.state_to_activate.name}.")
