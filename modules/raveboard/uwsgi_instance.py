import sys
import socketio
from raveboard.ui_context import UIContext

ctx = UIContext(*sys.argv[1:], skip_http_serve=True)
app = socketio.Middleware(ctx.sio)
ctx.run()
