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
        Thread(target=self.ui_serve_events_async).start()

    def ui_serve_events_async(self):
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
                    id=new_id,
                    signal=spike_or_act.id(),
                    parents=tuple(self.ui_model(parent).id for parent in parent_spikes))
            elif isinstance(spike_or_act, Activation):
                new_obj = UIActivationModel(
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
        # -- determine whether the UI should be concerned with this
        #  activation at all.
        act_model = self.ui_model(act)
        is_spiky = act.spiky()
        if not is_spiky and act_model.status == "wait":
            return
        update_needed = False

        # -- determine status transition
        if is_running:
            act_model.status = "run"
            update_needed = True
        else:
            act_is_ready = act.constraint.evaluate()
            if act_is_ready and act_model.status is "wait":
                act_model.status = "ready"
                update_needed = True

        # -- build spike reference structure, while
        #  determining whether the activations current one is up-to-date
        new_spike_ref_struct = []
        fully_unreferenced = True
        for i, conj in enumerate(act.constraint.conjunctions()):
            new_conj_dict = {}
            cur_conj_dict = {}
            new_spike_ref_struct.append(new_conj_dict)
            if len(act_model.spikes) > i:
                cur_conj_dict = act_model.spikes[i]
            else:
                update_needed = True
            for sig in conj.signals():
                spike_id = -1
                if sig.spike:
                    spike_id = self.ui_model(sig.spike).id
                    fully_unreferenced = False
                new_conj_dict[sig.name] = spike_id
                if sig.name not in cur_conj_dict or cur_conj_dict[sig.name] != spike_id:
                    update_needed = True

        # -- update spike ref structure, send activation to frontend on-demand
        if update_needed:
            act_model.spikes = new_spike_ref_struct
            self.sio.emit("activation", vars(act_model))

        # -- forget activation, if it was run and it is fully unreferenced
        if fully_unreferenced and act_model.status is "run":
            del self.ui_objects[act]

    # --------------------- Context Overrides ---------------------

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

    def run_once(self, seconds_passed=1., debug=False):
        super(UIContext, self).run_once(seconds_passed, debug)
        acts_to_update = (
            {act for act in self.ui_objects if isinstance(act, Activation)} |
            self._state_activations())
        for act in acts_to_update:
            self.ui_update_act(act)

    def _state_activated(self, act: Activation):
        super(UIContext, self)._state_activated(act)
        self.ui_update_act(act, is_running=True)
