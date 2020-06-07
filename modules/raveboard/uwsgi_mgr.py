from flask import Flask
from flask import request, redirect, send_from_directory, abort
from flask_cors import CORS
import os
import urllib.parse
import time
import yaml
import sys
import codecs
from raveboard.session import SessionManager

from reggol import get_logger
logger = get_logger(__name__)


# Valid config keys
RAVESTATE_SESSION_COMMAND = "ravestate_session_command"
SESSION_DB_PATH = "session_db_path"
SESSION_REFRESH_INTERVAL = "session_refresh_interval"
NUM_IDLE_SESSIONS = "num_idle_sessions"
USABLE_PORT_RANGE = "usable_port_range"
HOSTNAME = "hostname"
ZOMBIE_HEARTBEAT_THRESHOLD = "zombie_heartbeat_threshold"

if len(sys.argv) < 2 or not os.path.isfile(sys.argv[1]):
    logger.critical("The first argument does not point to a valid config file!")
    config = {}
else:
    with codecs.open(sys.argv[1]) as config_file:
        config: dict = yaml.load(config_file)
        if not isinstance(config, dict):
            logger.error("Config entries must be provided as a top-level yaml dictionary!")

# Record start timestamp to calculate uptime
start_timestamp = time.time()

# Instantiate flask app
#  TODO: More fine-grained CORS management, allowing only valid host:port combinations
app = Flask(__name__)
CORS(app)

# Child ravestate process call.
ravestate_session_command = config.pop(RAVESTATE_SESSION_COMMAND, [
    "{python}",
    "-m", "raveboard",
    "ravestate_hibye",
    "-d", "raveboard", "port",       "{port}",
    "-d", "raveboard", "session_db", "{session_db}",
    "-d", "raveboard", "announce",   "skip",
    "-d", "raveboard", "greet",      "connect"
])

# Session manager. Manages raveboard subprocesses.
sessions = SessionManager(
    db_path=config.pop(SESSION_DB_PATH, os.path.abspath("sessions.sqlite")),
    refresh_iv_secs=config.pop(SESSION_REFRESH_INTERVAL, 2.),
    session_launch_args=ravestate_session_command,
    num_idle_instances=config.pop(NUM_IDLE_SESSIONS, 2),
    usable_ports=set(range(*config.pop(USABLE_PORT_RANGE, [5010, 5020]))),
    hostname=config.pop(HOSTNAME, "localhost"),
    zombie_heartbeat_threshold=config.pop(ZOMBIE_HEARTBEAT_THRESHOLD, 30))


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
