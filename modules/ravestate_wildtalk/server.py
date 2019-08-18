from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, HTTPServer
import json

from .gpt2_backend import GPT2_Responder
from .convai_gpt_backend import ConvAI_GPT_Responder

from reggol import get_logger
logger = get_logger(__name__)

responder = None


class S(BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/json')
        self.send_header('x-dino', 'dinos are great')
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        query_components = parse_qs(urlparse(self.path).query)
        for key in query_components:
            query_components[key] = ''.join(query_components[key])
        prompt = query_components.pop("prompt")
        response = responder.process(prompt, query_components)
        json_payload = json.dumps({"response": response})
        self.wfile.write(bytes(json_payload, encoding="utf-8"))

    def do_HEAD(self):
        self._set_headers()


def run(port=5100, model="convai_gpt"):
    responder_map = {"convai_gpt": ConvAI_GPT_Responder, "gpt2": GPT2_Responder}
    global responder
    if model == "parlai":
        from .parlai_backend import Parlai_Responder
        responder = Parlai_Responder()
    else:
        responder = responder_map[model]()
    server_address = ('', port)
    httpd = HTTPServer(server_address, S)
    logger.info(f'Started {model} Server')
    httpd.serve_forever()
