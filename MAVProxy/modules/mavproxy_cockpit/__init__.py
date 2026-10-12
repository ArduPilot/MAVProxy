#!/usr/bin/env python

'''
MAVProxy Cockpit module that serves files from its directory
'''

from MAVProxy.modules.lib import mp_module
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading

class CustomHTTPRequestHandler(SimpleHTTPRequestHandler):
    """Custom handler that serves from absolute directory path"""
    def __init__(self, *args, directory=None, **kwargs):
        # SimpleHTTPRequestHandler supports absolute paths directly
        super().__init__(*args, directory=directory, **kwargs)

class Cockpit(mp_module.MPModule):
    def __init__(self, mpstate):
        super(Cockpit, self).__init__(mpstate, "cockpit", "Cockpit GCS module")

        # Default port
        self.port = 8080
        self.output_str = 'wsserver:0.0.0.0:5010'

        # Get the directory where this module file is located
        self.module_dir = os.path.dirname(os.path.abspath(__file__))
        self.html_dir = os.path.join(self.module_dir, 'static', 'dist')

        # Start server automatically
        self.start_server()
        self.setup_output()

    def start_server(self):
        '''Start the web server serving from module directory'''
        try:
            # Create handler class with bound directory
            handler = lambda *args, **kwargs: CustomHTTPRequestHandler(*args,
                                                                     directory=self.html_dir,
                                                                     **kwargs)

            # Create and start server
            self.web_server = HTTPServer(('', self.port), handler)
            self.server_thread = threading.Thread(target=self.web_server.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
            print(f"\n\nWeb server running at http://localhost:{self.port}/?mainConnectionURI=ws://localhost:5010#/\n\n")

        except Exception as e:
            print(f"Failed to start server: {e}")

    def setup_output(self):
        '''Setup a new MAVLink output'''

        self.module('output').cmd_output(['add', self.output_str])
        print(f"Added new MAVLink output: {self.output_str}")

    def unload(self):
        '''Unload module'''
        # Stop the web server
        if hasattr(self, 'web_server'):
            self.web_server.shutdown()
            self.web_server.server_close()

        # Remove the WebSocket output
        self.module('output').cmd_output(['remove', self.output_str])
        print(f"Removed MAVLink output: {self.output_str}")

def init(mpstate):
    '''Initialize module'''
    return Cockpit(mpstate)