# Dialogic context class


import importlib
from threading import Thread, Lock, Semaphore

from dialogic import icontext
from dialogic import activation
from dialogic import module
from dialogic import state
from dialogic import property
from dialogic import registry


class Context(icontext.IContext):

    default_signals = (":startup", ":shutdown", ":idle")
    default_property_signals = (":changed", ":pushed", ":popped", ":deleted")

    def __init__(self, configfile: str=""):
        self.signal_queue = []
        self.signal_queue_lock = Lock()
        self.signal_queue_counter = Semaphore(0)
        self.run_task = None
        self.shutdown_flag = False
        self.properties = {}
        self.activation_candidates = dict()

        self.states = set()
        self.states_per_signal = {signal_name: set() for signal_name in self.default_signals}
        self.states_lock = Lock()

    def emit(self, signal_name: str):
        with self.signal_queue_lock:
            self.signal_queue.append(signal_name)
            self.signal_queue_counter.release()

    def run(self):
        if self.run_task:
            print("Attempt to start context twice!")
            return
        self.run_task = Thread(target=self._run_private)
        self.run_task.start()
        self.emit(":startup")

    def shutting_down(self):
        return self.shutdown_flag

    def shutdown(self):
        self.shutdown_flag = True
        self.emit(":shutdown")
        self.run_task.join()

    def add_module(self, module_name: str):
        if registry.has_module(module_name):
            self._module_registration_callback(registry.get_module(module_name))
            return
        registry.import_module(module_name=module_name, callback=self._module_registration_callback)

    def add_state(self, *, mod: module.Module, st: state.State):
        if st in self.states:
            print("Attempt to add state `{}` twice!".format(st.name))
            return

        # annotate the state's signal name with it's module name
        if len(st.signal) > 0:
            st.signal = "{}:{}".format(mod.name, st.signal)
        st.module_name = mod.name

        # make sure that all of the state's depended-upon properties exist
        for prop in st.read_props+st.write_props:
            if prop not in self.properties:
                print("Attempt to add state which depends on unknown property `{}`!".format(prop))

        # register the state's signal
        with self.states_lock:
            if st.signal:
                self.states_per_signal[st.signal] = set()
            # make sure that all of the state's depended-upon signals exist
            for clause in st.triggers:
                for signal in clause:
                    if signal in self.states_per_signal:
                        self.states_per_signal[signal].add(st)
                    else:
                        print("Attempt to add state which depends on unknown signal `{}`!".format(signal))
            self.states.add(st)

    def rm_state(self, *, st: state.State):
        if st not in self.states:
            print("Attempt to remove unknown state `{}`!".format(st.name))
            return
        with self.states_lock:
            if st.signal:
                self.states_per_signal.pop(st.signal)
            for clause in st.triggers:
                for signal in clause:
                    self.states_per_signal[signal].remove(st)
            self.states.remove(st)

    def add_prop(self, *, mod: module.Module, prop: property.PropertyBase):
        if prop.name in self.properties.values():
            print("Attempt to add property `{}` twice!".format(prop.name))
            return
        # prepend module name to property name
        prop.module_name = mod.name
        # register property
        self.properties[prop.fullname()] = prop
        # register all of the property's signals
        with self.states_lock:
            for signal in self.default_property_signals:
                self.states_per_signal[prop.fullname()+signal] = set()

    def __setitem__(self, key, value):
        pass

    def __getitem__(self, key):
        return self.properties[key]

    def _module_registration_callback(self, mod: module.Module):
        for st in mod.states:
            self.add_state(mod=mod, st=st)
        for prop in mod.props:
            self.add_prop(mod=mod, prop=prop)

    def _run_private(self):
        while not self.shutdown_flag:
            # TODO: Recognize and signal Idle-ness
            self.signal_queue_counter.acquire()
            with self.signal_queue_lock:
                signal_name = self.signal_queue.pop(0)

            # collect states which depend on the new signal,
            # and create state activation objects for them if necessary
            print ("Received {} ...".format(signal_name))
            with self.states_lock:
                for state in self.states_per_signal[signal_name]:
                    if state.name not in self.activation_candidates:
                        self.activation_candidates[state.name] = activation.StateActivation(state, self)

            print ("State activation candidates: \n"+"\n".join(
                "- "+state_name for state_name in self.activation_candidates))

            current_activation_candidates = self.activation_candidates
            self.activation_candidates = dict()
            consider_for_immediate_activation = []

            # go through candidates and remove those which want to be removed,
            # remember those which want to be remembered, forget those which want to be forgotten
            for state_name, act in current_activation_candidates.items():
                notify_return = act.notify_signal(signal_name)
                print ("-> {} returned {} on notify_signal {}".format(act.state_to_activate.name, notify_return, signal_name))
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
                    print ("-> Activating {}".format(act.state_to_activate.name))
                    thread = act.run()
                    thread.start()
                else:
                    print ("-> Dropping activation of `{}`.".format(act.state_to_activate.name))
