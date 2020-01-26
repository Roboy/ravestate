from flask import Flask
from flask import request, redirect, send_from_directory, abort
from flask_cors import CORS
import os
import urllib.parse
import time

# Record start timestamp to calculate uptime
start_timestamp = time.time()

# Instantiate flask app
#  TODO: More fine-grained CORS management, allowing only valid host:port combinations
app = Flask(__name__)
CORS(app)

# Import after flask such that custom logging is not destroyed
from raveboard.ui_context import PORT_CONFIG_KEY, SESSION_DB_KEY, URL_ANNOUNCE_KEY, GREETING_KEY, GREET_ON_CONNECT, RAVEBOARD
from raveboard.session import Session, SessionManager

# Get path to python interpreter, such that we can use it to launch ravestate
py_interpreter = os.path.join(os.__file__.split("lib/")[0], "bin", "python")

# Session database path. Maintains a sessions table with cols. `port`, `state`, `sio_uri`, `last_used`, `secret`
session_db_path = os.path.abspath("sessions.sqlite")

# Child ravestate process call.
#  TODO: Convert to uwsgi
ravestate_session_command = [
    py_interpreter,
    "-m", RAVEBOARD,
    # "ravestate_wildtalk",
    # "ravestate_persqa",
    # "ravestate_roboyqa",
    # "ravestate_fillers",
    "ravestate_hibye",
    "-d", RAVEBOARD, PORT_CONFIG_KEY, "{port}",
    "-d", RAVEBOARD, SESSION_DB_KEY, session_db_path,
    "-d", RAVEBOARD, URL_ANNOUNCE_KEY, "skip",
    "-d", RAVEBOARD, GREETING_KEY, GREET_ON_CONNECT]

# Session manager. Manages raveboard subprocesses.
sessions = SessionManager(
    db_path=session_db_path,
    refresh_iv_secs=2.,
    session_launch_args=ravestate_session_command,
    num_idle_instances=2,
    usable_ports=set(range(5010, 5020)),
    hostname="localhost",
    zombie_heartbeat_threshold=30)


@app.route('/', methods=['GET'])
@app.route('/index.html', methods=['GET'])
def hello():

    # check if authorized
    if "rs-sio-url" in request.args and "token" in request.args:
        rs_sio_url = request.args["rs-sio-url"]
        token = request.args["token"]
        if sessions.is_authorized(rs_sio_url, token):
            response = send_from_directory("dist/ravestate", "index.html")
            response.headers['Cache-Control'] = 'no-store'
            return response
        else:
            pass  # auth token mismatch -> serve a new session

    # TODO: Make DDoS a little bit harder
    # take a free port
    if not sessions.has_idle_session():
        return abort(503, "No free sessions available. Please come back in a few minutes.")

    # instantiate new session
    new_session = sessions.pop_idle_session()

    # forward with rs-sio-url & token
    return redirect(f"/index.html?rs-sio-url={urllib.parse.quote(new_session.sio_url)}&token={new_session.secret}")


@app.route('/<path:path>')
def static_file(path):
    return send_from_directory("dist/ravestate", path)


@app.route('/stats')
def stats():
    return f"""
    <h2>Purring like a kitten.</h2>
    <h3>Sweet stats:</h3>
    <ul>
        <li>Uptime: {int(time.time() - start_timestamp)}s
        <li>Total sessions served: {sessions.created_sessions}
    </ul>
    <h3>Active sessions:</h3>
    <ul>
        {"".join(f"<li><b>Port:</b> {int(sess[0])}, <b>Uptime:</b> {int(sess[1])}s, <b>Status:</b> {sess[2]}" for sess in sessions.sessions())}
    </ul>
    """


if __name__ == '__main__':
    app.run()
