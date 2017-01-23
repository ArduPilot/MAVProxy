


import threading, sys, traceback

# Import dumpstacks to install a SIGQUIT handler that shows a stack dump for all stacks
# From http://stackoverflow.com/questions/132058/showing-the-stack-trace-from-a-running-python-application

def dumpstacks(signal, frame):
    id2name = dict([(th.ident, th.name) for th in threading.enumerate()])
    code = []
    for threadId, stack in sys._current_frames().items():
        code.append("\n# Thread: %s(%d)" % (id2name.get(threadId, ""), threadId))
        for filename, lineno, name, line in traceback.extract_stack(stack):
            code.append('File: "%s", line %d, in %s' % (filename, lineno, name))
            if line:
                code.append("  %s" % (line.strip()))
    print("\n".join(code))

try:
    import signal
    signal.signal(signal.SIGQUIT, dumpstacks)
except Exception as e:
    # Silently ignore failures installing this handler (probably won't work on Windows)
    pass
