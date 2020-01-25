from flask import Flask
from flask import request, redirect, send_from_directory, abort
from flask_cors import CORS
import os
import urllib.parse
from raveboard.session import Session, SessionManager

# Get path to python interpreter, such that we can use it to launch ravestate
py_interpreter = os.path.join(os.__file__.split("lib/")[0], "bin", "python")

# Session database path. Maintains a sessions table with cols. `port`, `state`, `sio_uri`, `last_used`, `secret`
session_db_path = os.path.abspath("sessions.sqlite")

# Child ravestate process call.
#  TODO: Convert to uwsgi
#  TODO: Pass on auth token, session tracking DB path
ravestate_session_command = [
    py_interpreter,
    "-m", "raveboard",
    # "ravestate_wildtalk",
    # "ravestate_persqa",
    # "ravestate_roboyqa",
    # "ravestate_fillers",
    "ravestate_hibye",
    "-d",
    "raveboard",
    "port",
    "{port}"]

# Session manager. Manages raveboard subprocesses.
sessions = SessionManager(
    db_path=session_db_path,
    refresh_iv_ms=1000,
    session_launch_args=ravestate_session_command,
    num_idle_instances=2,
    usable_ports=set(range(5010, 5020)),
    hostname="localhost")

# Instantiate flask app
#  TODO: More fine-grained CORS management, allowing only valid host:port combinations
app = Flask(__name__)
CORS(app)


@app.route('/')
@app.route('/index.html')
def hello():

    # check if authorized
    if "rs-sio-url" in request.args and "token" in request.args:
        rs_sio_url = request.args["rs-sio-url"]
        token = request.args["token"]
        if sessions.is_authorized(rs_sio_url, token):
            return send_from_directory("dist/ravestate", "index.html")
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


if __name__ == '__main__':
    app.run()
