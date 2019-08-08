from flask import Flask
from flask import jsonify
from flask_socketio import SocketIO, emit
import threading

app = Flask(__name__, static_url_path='')

global socketio, lastTick, lastSpike
socketio = SocketIO(app)


@app.route('/tick')
def get_sample_tick_data():

    sample_unfulfilled_activation_data = {}
    # activation id, unique through frontend & context
    sample_unfulfilled_activation_data['id'] = 0
    sample_unfulfilled_activation_data['name'] = 'sample activation 0'
    sample_unfulfilled_activation_data['specificity'] = 5
    sample_unfulfilled_activation_data['activated'] = False

    constraints = {}
    constraints['sample spike 0'] = 0
    constraints['sample nonexistent spike'] = -1

    sample_unfulfilled_activation_data["constraints"] = constraints

    sample_all_activations_data = {}
    sample_all_activations_data["activations"] = (sample_unfulfilled_activation_data, )
    sample_all_activations_data["type"] = "tick"

    return jsonify(sample_all_activations_data)

@app.route('/spike')
def get_sample_spike_data():
    sample_new_spike = {}

    # spike id, unique through frontend & context
    sample_new_spike['id'] = 0
    sample_new_spike['type'] = "spike"
    sample_new_spike["property_value_at_creation"] = "whatever you want it to be"
    sample_new_spike["parent"] = 1

    sample_new_spike['name'] = 'sample spike 0'

    # the signal this spike corresponds to
    # TODO discuss - this can be used to connect activations and spikes, alternative is 'satisfied' in tick update
    # sample_new_spike['signal'] = 2

    return jsonify(sample_new_spike)


def tick(data):
    global lastTick
    lastTick = data
    emit('tick', jsonify(data))


def spike(data):
    global lastSpike
    lastSpike = data
    emit('spike', jsonify(data))


def advertise(*, ip="0.0.0.0", port=5001, debug=False):
    app.debug=debug
    appthread = threading.Thread(target=app.run, args=(ip, port))
    appthread.start()
    socketthread = threading.Thread(target=socketio.run, args=(app,))
    socketthread.start()
