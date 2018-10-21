# Dialogic class which encapsualtes the activation of a single state

from dialogic import state
from dialogic import wrappers
from dialogic import isession
from threading import Thread


class StateActivation:

    def __init__(self, st: state.State, sess: isession.ISession):
        self.state_to_activate = st
        self.unfulfilled = [set(trigger_clause) for trigger_clause in st.triggers]
        self.sess = sess

    def specificity(self):
        # TODO: Calculate specificity properly
        return 1.

    def notify_signal(self, signal_name: str):
        for unfulfilled_trigger in self.unfulfilled:
            if signal_name in unfulfilled_trigger:
                unfulfilled_trigger.remove(signal_name)
                if len(unfulfilled_trigger) == 0:
                    return 1
        return 0

    def run(self):
        return Thread(target=self._run_private)

    def _run_private(self):
        session_wrapper = wrappers.SessionWrapper(self.sess, self.state_to_activate)
        result = self.state_to_activate(session_wrapper)
        if result == session_wrapper.EmitSignal and self.state_to_activate.signal:
            self.sess.emit(self.state_to_activate.signal)
        if result == session_wrapper.DeleteMe:
            self.sess.rm_state(st=self.state_to_activate)
