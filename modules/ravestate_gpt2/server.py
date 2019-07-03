from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, HTTPServer
import json

from ravestate_gpt2.gpt2_backend import GPT2Responder

from reggol import get_logger
logger = get_logger(__name__)


class S(BaseHTTPRequestHandler):
    gpt2_responder = GPT2Responder()

    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/json')
        self.send_header('x-dino', 'dinos are great')
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        query_components = parse_qs(urlparse(self.path).query)
        prompt = ''.join(query_components["prompt"])
        response = self.gpt2_responder.process(prompt)
        json_payload = json.dumps({"response": response})
        self.wfile.write(bytes(json_payload, encoding="utf-8"))

    def do_HEAD(self):
        self._set_headers()


def run(server_class=HTTPServer, handler_class=S, port=5100):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    logger.info('Started GPT2 Server')
    httpd.serve_forever()
