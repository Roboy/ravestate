from flask import Flask, url_for, render_template, send_from_directory
from flask import jsonify

from ravestate import registry

app = Flask(__name__)
context = None

@app.route('/')
def index():
    global context
    return render_template('index.html')

@app.route('/data')
def get_data():
    global context
    sets = []
    for module in registry._registered_modules.items():
        for state in module[1].states:
            for trigger in state.read_props:
                set = {}
                set['source'] = " " + str(trigger)
                set['target'] = state.name
                set['type'] = 'notifies'
                sets.append(set)
            for target in state.write_props:
                set = {}
                set['source'] = state.name
                set['target'] = target
                set['type'] = 'changes'
                sets.append(set)
    # for prop in context._properties.keys():
    #     for type in context.default_property_signals:
    #         set = {}
    #         set['source'] = prop
    #         set['target'] = "{}{}".format(prop, type)
    #         set['type'] = 'creates'
    #         sets.append(set)

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


def advertise(*, ctx, ip="0.0.0.0", port=5000, debug=False):
    global context
    context = ctx
    app.debug=debug
    app.run(ip, port)
    url_for('static', filename='graph.css')
