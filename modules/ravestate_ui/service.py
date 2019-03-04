from flask import Flask, url_for, render_template, send_from_directory
from flask import jsonify
from flask_socketio import SocketIO, emit

from ravestate.module import Module

app = Flask(__name__, static_url_path='')

context = None
socketio = SocketIO(app)

@app.route('/scripts/<path:path>')
def send_js(path):
    return send_from_directory('scripts', path)

@app.route('/node_modules/<path:path>')
def send_node(path):
    return send_from_directory('node_modules', path)

@app.route('/')
def index():
    global context
    return render_template('index.html')

@app.route('/matter')
def matter_demo():
    global context
    return render_template('matter-demo.html')

@app.route('/data')
def get_data():
    global context
    sets = parse_data()
    return jsonify(sets)


def parse_data():
    sets = []
    for name, module in Module.registered_modules.items():
        for state in module.states:
            for trigger in state.read_props:
                set = {}
                set['source'] = str(" " + trigger)
                set['target'] = state.name
                set['type'] = 'notifies'
                sets.append(set)
            for target in state.write_props:
                set = {}
                set['source'] = state.name
                set['target'] = target
                set['type'] = 'changes'
                sets.append(set)
    return sets


# background process happening without any refreshing
@app.route('/background_process_test')
def background_process_test():
    print("Hello")
    return "nothing"

@socketio.on('update')
def update():
    sets = parse_data()
    emit('newdata', jsonify(sets))

def advertise(*, ctx, ip="0.0.0.0", port=5000, debug=False):
    global context
    context = ctx
    app.debug=debug
    app.run(ip, port)
    socketio.run(app)
    url_for('static', filename='graph.css')
