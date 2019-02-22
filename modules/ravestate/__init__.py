
if __name__ == "main":

    from ravestate.context import Context
    import sys

    ctx = Context(*sys.argv[1:])
    ctx.run()
