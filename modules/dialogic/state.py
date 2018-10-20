# Dialogic State-related definitions


class State:

    def __init__(self, *, signal, write, read, triggers, action):
        assert(callable(action))
        self.name = "{}{}".format(action.__name__, " (signal {})" if len(signal) > 0 else " (not signaling)")

        # catch the insane case
        if not len(read) and not len(triggers):
            print(
                "The state `{}` is not reading any properties, nor waiting for any triggers. ".format(self.name) +
                "It will never be activated!")

        # convert read/write properties to tuples
        if isinstance(write, str):
            write = (write,)
        if isinstance(read, str):
            read = (read,)

        # listen to default changed-signals if no signals are given.
        # convert triggers to disjunctive normal form.
        if not len(triggers):
            triggers = tuple(("{}:changed".format(rprop_name),) for rprop_name in read)
        else:
            if isinstance(triggers, str):
                triggers = (triggers,)
            if isinstance(triggers[0], str):
                triggers = (triggers,)

        self.signal = signal
        self.write_props = write
        self.read_props = read
        self.triggers = triggers
        self.action = action

    def __call__(self, session):
        return self.action(session)


def state(*, signal: str="", write: tuple=(), read: tuple=(), triggers: tuple=()):
    """
    Decorator to declare a new state, which may emit a certain signal,
    write to a certain set of properties (calling set, push, pop, delete),
    and read from certain properties (calling read).
    """
    def state_decorator(action):
        nonlocal signal, write, read, triggers
        return State(signal=signal, write=write, read=read, triggers=triggers, action=action)
    return state_decorator


if __name__ == '__main__':

    # test basic decorating annotation
    @state(signal="test-signal", read="myprop", write="myprop", triggers=":idle")
    def test_state(session):
        return "Hello world!"
    assert(test_state.signal == "test-signal")
    assert(test_state.read_props == ("myprop",))
    assert(test_state.triggers == ((":idle",),))
    assert(test_state(None) == "Hello world!")

    # test whether default signals are assigned
    @state(read="myprop")
    def test_default_trigger_assignment(session):
        return "Hello universe!"
    assert(test_default_trigger_assignment.triggers == (("myprop:changed",),))

    @state()
    def insane_state():
        print("You will never see this.")

    print(test_state(None))
