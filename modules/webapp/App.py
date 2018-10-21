from flask import Flask, url_for, render_template, send_from_directory
from test import sess
from flask import jsonify

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def get_data():
    sets = []
    for state in sess.states:
        for trigger in state.triggers:
            set = {}
            set['source'] = trigger[0]
            set['target'] = state.name
            set['type'] = 'notifies'
            sets.append(set)
    for prop in sess.properties.keys():
        for type in sess.default_property_signals:
            set = {}
            set['source'] = prop
            set['target'] = "{}{}".format(prop, type)
            set['type'] = 'creates'
            sets.append(set)
    return jsonify(sets)

app.debug=True
app.run("0.0.0.0", 5000)
url_for('static', filename='graph.css')
