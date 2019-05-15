import sys
from ravestate_ui.ui_context import UIContext

ctx = UIContext(*sys.argv[1:])
ctx.run()
