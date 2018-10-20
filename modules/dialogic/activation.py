# Dialogic class which encapsualtes the activation of a single state

from dialogic import state
from dialogic import wrappers
from dialogic import isession
from threading import Thread


class StateActivation:

    def __init__(self, st: state.State, sess: isession.ISession):
        self.state_to_activate = st
        self.unfulfilled = set(st.triggers[0])
        self.state_to_activate = st
        self.sess = sess

    def specificity(self):
        # TODO: Calculate specificity properly
        return 1.

    def notify_signal(self, signal_name: str):
        self.unfulfilled.remove(signal_name)
        return 0 if len(self.unfulfilled) > 0 else 1

    def run(self):
        return Thread(target=self._run_private)

    def _run_private(self):
        session_wrapper = wrappers.SessionWrapper(self.sess, self.state_to_activate)
        if self.state_to_activate(session_wrapper) and self.state_to_activate.signal:
            self.sess.emit(self.state_to_activate.signal)