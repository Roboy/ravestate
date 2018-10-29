from flask import Flask, url_for, render_template, send_from_directory
from flask import jsonify

app = Flask(__name__)
session = None

@app.route('/')
def index():
    global session
    return render_template('index.html')

@app.route('/data')
def get_data():
    global session
    sets = []
    for state in session.states:
        for trigger in state.triggers:
            set = {}
            set['source'] = trigger[0]
            set['target'] = state.name
            set['type'] = 'notifies'
            sets.append(set)
        for target in state.write_props:
            set = {}
            set['source'] = state.name
            set['target'] = target
            set['type'] = 'changes'
            sets.append(set)
    for prop in session.properties.keys():
        for type in session.default_property_signals:
            set = {}
            set['source'] = prop
            set['target'] = "{}{}".format(prop, type)
            set['type'] = 'creates'
            sets.append(set)

    cleanSet = []
    for set in sets: # Don't display unused signals
        if set['type'] == 'creates':
            used = False
            for compare in sets:
                if set['target'] == compare['source']:
                    used = True
                    break
            if used:
                cleanSet.append(set)
        else:
            cleanSet.append(set)
    return jsonify(cleanSet)


def advertise(*, sess, ip="0.0.0.0", port=5000, debug=False):
    global session
    session = sess
    app.debug=debug
    app.run(ip, port)
    url_for('static', filename='graph.css')
