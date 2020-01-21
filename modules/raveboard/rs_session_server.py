from flask import Flask
from flask import request
from typing import Dict, List
app = Flask(__name__, static_folder="dist/ravestate")

# Secret tokens per socketio-uri
current_tokens: Dict[str, str] = {}

# Ports for which no active ravestate instance is running
free_ports: List[int] = list(range(5001, 5010))


@app.route('/')
def hello():

    if "rs-sio-uri" in request.args and "token" in request.args:

        rs_sio_uri = request.args["rs-sio-uri"]
        token = request.args["token"]

        if rs_sio_uri in current_tokens and current_tokens[rs_sio_uri] == token:

            return

    # take a free port

    # start ravestate on the selected port
    # optional: Create a secret that must be known by port users.

    # forward with rs-sio-uri & token
    pass


if __name__ == '__main__':
    app.run()
