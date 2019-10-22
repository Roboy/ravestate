import ravestate as rs
import ravestate_rawio as rawio

from .ui_model import UIActivationModel, UISpikeModel

from typing import Any, Tuple, List, Union, Dict
import socketio
import flask
import urllib.parse
from threading import Thread, Lock
from collections import namedtuple, defaultdict

from reggol import get_logger
logger = get_logger(__name__)

RAVEBOARD = "raveboard"
PORT_CONFIG_KEY = "port"
URL_PREFIX_KEY = "host"

RAVEBOARD_CONFIG = {
    PORT_CONFIG_KEY: 42424,
    URL_PREFIX_KEY: "http://localhost"
}


class UIContext(rs.Context):

    def __init__(self, *arguments, runtime_overrides: List[Tuple[str, str, Any]] = None):
        self.msgs_lock = Lock()
        self.sio = socketio.Server(cors_allowed_origins="*", async_mode="threading")
        self.next_id_for_object = defaultdict(int)
        self.ui_objects: Dict[Union[rs.Spike, rs.Activation], Union[UISpikeModel, UIActivationModel]] = dict()

        super().__init__(*arguments, runtime_overrides=runtime_overrides)
        Thread(target=self.ui_serve_events_async).start()

    def ui_serve_events_async(self):
        app = flask.Flask(__name__, static_folder="dist/ravestate")
        app.wsgi_app = socketio.Middleware(self.sio, app.wsgi_app)
        app.run(host='0.0.0.0', port=self.conf(mod=RAVEBOARD, key=PORT_CONFIG_KEY), threaded=True)

    def _load_modules(self, modules: List[str]):
        super()._load_modules(modules)

        with rs.Module(name=RAVEBOARD, config=RAVEBOARD_CONFIG, depends=(rawio.mod,)) as mod:

            @rs.state(cond=rs.sig_startup.detached().min_age(1.), write=rawio.prop_out)
            def announce_ui_url(ctx):
                sio_uri = urllib.parse.quote(f"{ctx.conf(key=URL_PREFIX_KEY)}:{ctx.conf(key=PORT_CONFIG_KEY)}")
                url = f"{ctx.conf(key=URL_PREFIX_KEY)}:{ctx.conf(key=PORT_CONFIG_KEY)}/ravestate/index.html?rs-sio-url={sio_uri}"
                logger.info(f"Raveboard URL: {url}")
                ctx[rawio.prop_out] = f"Watch your conversation on Raveboard here! {url}"

            @rs.state(read=rawio.prop_out)
            def emit_output(ctx):
                self.sio.emit("output", {"type": "output", "text": ctx[rawio.prop_out.changed()]})

        self._add_ravestate_module(mod)

        @rs.receptor(ctx_wrap=self, write=(rawio.prop_in,))
        def receive_input(ctx, _, new_input_event):
            if 'text' not in new_input_event:
                logger.error("Bad socket.io message for input event!")
                return
            ctx[rawio.prop_in] = new_input_event['text']

        self.sio.on("input", receive_input)

    def ui_model(self, spike_or_act: Union[rs.Spike, rs.Activation], parent_spikes=()) -> Union[UIActivationModel, UISpikeModel]:
        if spike_or_act in self.ui_objects:
            return self.ui_objects[spike_or_act]
        else:
            new_id = self.next_id_for_object[spike_or_act.__class__]
            self.next_id_for_object[spike_or_act.__class__] += 1
            new_obj = None
            if isinstance(spike_or_act, rs.Spike):
                new_obj = UISpikeModel(
                    id=new_id,
                    signal=spike_or_act.id(),
                    parents=tuple(self.ui_model(parent).id for parent in parent_spikes))
            elif isinstance(spike_or_act, rs.Activation):
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

    def ui_update_act(self, act: rs.Activation, is_running=False):
        # -- do not report on boring activations
        if act not in self.ui_objects and not act.spiky(filter_boring=True):
            return

        act_model = self.ui_model(act)
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
            elif not act_is_ready and act_model.status is "ready":
                act_model.status = "wait"
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
                    spike_model = self.ui_model(sig.spike)
                    spike_id = spike_model.id
                    if not spike_model.published:
                        self.sio.emit("spike", vars(spike_model))
                        spike_model.published = True
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

    def emit(self, signal, parents=None, wipe: bool = False, payload=None, boring=False) -> rs.Spike:
        new_spike = super().emit(signal, parents, wipe, payload, boring)
        # create spike ui model, but only send it on demand when it is ref'd by an activation
        #  exception: the spike is an offspring spike
        spike_model = self.ui_model(new_spike, parent_spikes=parents if parents else ())
        if parents:
            self.sio.emit("spike", vars(spike_model))
            spike_model.published = True
        return new_spike

    def run_once(self, seconds_passed=1., debug=False):
        super(UIContext, self).run_once(seconds_passed, debug)
        with self._lock:
            acts_to_update = (
                {act for act in self.ui_objects if isinstance(act, rs.Activation)} |
                self._state_activations())
        for act in acts_to_update:
            self.ui_update_act(act)


    def _state_activated(self, act: rs.Activation):
        super(UIContext, self)._state_activated(act)
        self.ui_update_act(act, is_running=True)
