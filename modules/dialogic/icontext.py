# Interface for the context (needed to preempt cyclic dependencies)


from dialogic import property
from dialogic import module
from dialogic import state


class IContext:

    def emit(self, signal_name: str):
        pass

    def add_state(self, *, mod: module.Module, st: state.State):
        pass

    def rm_state(self, *, st: state.State):
        pass

    def add_prop(self, *, mod: module.Module, prop: property.PropertyBase):
        pass

    def shutting_down(self):
        pass

    def shutdown(self):
        pass

    def __setitem__(self, key, value):
        pass

    def __getitem__(self, key):
        pass
