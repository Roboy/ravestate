from ravestate.context import Context
from ravestate.spike import Spike
from ravestate.activation import Activation
from .ui_model import UIActivationModel, UISpikeModel

from typing import Any, Tuple, List, Union, Dict
import socketio
import flask
from threading import Thread, Lock
from collections import namedtuple, defaultdict

from reggol import get_logger
logger = get_logger(__name__)


class UIContext(Context):

    def __init__(self, *arguments, runtime_overrides: List[Tuple[str, str, Any]] = None):
        super().__init__(*arguments, runtime_overrides=runtime_overrides)
        self.msgs_lock = Lock()
        self.sio = socketio.Server(cors_allowed_origins="*", async_mode="threading")
        self.next_id_for_object = defaultdict(int)
        self.ui_objects: Dict[Union[Spike, Activation], Union[UISpikeModel, UIActivationModel]] = dict()
        Thread(target=self._serve_events_async).start()

    def _serve_events_async(self):
        app = flask.Flask(__name__)
        app.wsgi_app = socketio.Middleware(self.sio, app.wsgi_app)
        app.run(port=4242, threaded=True)

    def ui_model(self, spike_or_act: Union[Spike, Activation], parent_spikes=()) -> Union[UIActivationModel, UISpikeModel]:
        if spike_or_act in self.ui_objects:
            return self.ui_objects[spike_or_act]
        else:
            new_id = self.next_id_for_object[spike_or_act.__class__]
            self.next_id_for_object[spike_or_act.__class__] += 1
            new_obj = None
            if isinstance(spike_or_act, Spike):
                new_obj = UISpikeModel(
                    type="spike",
                    id=new_id,
                    signal=spike_or_act.id(),
                    parents=tuple(self.ui_model(parent).id for parent in parent_spikes))
            elif isinstance(spike_or_act, Activation):
                new_obj = UIActivationModel(
                    type="activation",
                    id=new_id,
                    state=spike_or_act.state_to_activate.name,
                    specificity=1.,
                    status="wait",
                    spikes=[])
            else:
                logger.error(f"Attempt to retrieve UI model for unknown object {spike_or_act}!")
            self.ui_objects[spike_or_act] = new_obj
            return new_obj

    def ui_update_act(self, act: Activation, is_running=False):
        pass

    def emit(self, signal, parents=None, wipe: bool=False, payload=None) -> None:
        # Copy instead of calling super to obtain reference to created spike.
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
            new_spike_model = self.ui_model(new_spike, parent_spikes=parents if parents else ())
            self.sio.emit("spike", vars(new_spike_model))

    def _state_activated(self, act: Activation):
        super(UIContext, self)._state_activated(act)
        self.ui_update_act(act)
