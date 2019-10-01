

class UIActivationModel:
    """
    Model which represents a spike, as seen by the ravestate UI.
    """
    def __init__(self, *, id, state, specificity, status, spikes):
        self.type = "activation"
        self.id = id
        self.state = state
        self.specificity = specificity
        self.status = status
        self.spikes = spikes


class UISpikeModel:
    """
    Model which represents an activation, as seen by the ravestate UI.
    """
    def __init__(self, *, id, signal, parents):
        self.type = "spike"
        self.id = id
        self.signal = signal
        self.parents = parents
        self.published = False
