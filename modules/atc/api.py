#!/usr/bin/env python
''' API for interacting with pyke

    This is in a separate module so it's more convenient to test
    and because maybe I will want to use it outside MAVProxy one day,
    such as with an ivy-python and a paparazzi system.
'''

import sys
from pyke import knowledge_engine, krb_traceback, ask_tty

engine = knowledge_engine.engine(__file__)
#engine.ask_module = ask_tty # probably need to hack to mavproxy here

def preflight():
    engine.reset()
    try:
        engine.activate('preflight')
        engine.prove_1_goal('preflight.clear_to_takeoff()')
    except StandardError:
        krb_traceback.print_exc()
        sys.exit(1)


''' Ideas:

Interogate the operator.
        
This might be a thread. It is a kind of session, obtaining the 
nesiscary knowledge to acomplish some goal.

I expect that when  it comes to deploying generated agents I will
probably need an arbitrary collection of threads (because I might want
concurrent agents with individual responsabilities).

Before then, I might want one thread for each of:

 * Interogating the operator (this thread).
 * Proof on demand - backward-chaining proofs [and generating 
   artefacts] based on the module's current knowledge.

And then the beuro of generated agents. The main program will manage
the beuro, no need for a separate thread for that.
'''

# self-checking, debug
if __name__ == '__main__':
    preflight()
