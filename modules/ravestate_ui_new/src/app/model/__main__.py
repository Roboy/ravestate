import sys
from ravestate_ui_new.src.app.model.ui_context import UIContext

ctx = UIContext(*sys.argv[1:])
ctx.run()
