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
        set = {}
        for prop in state.triggers:
            set['source'] = prop[0]
            set['target'] = state.name
            set['type'] = 'licencing'
        sets.append(set)
    return jsonify(sets)

app.debug=True
app.run("0.0.0.0", 5000)
url_for('static', filename='w3.css')
url_for('static/js', filename='bubblechart.js')