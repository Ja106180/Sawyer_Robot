#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import http.server
import socketserver
import threading
import json

class MyHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

    def do_GET(self):
        if self.path == '/api/actions':
            package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            actions_dir = os.path.join(package_path, 'actions')
            actions = []
            if os.path.exists(actions_dir):
                for f in os.listdir(actions_dir):
                    if f.endswith('.csv'):
                        actions.append(f[:-4])
            actions.sort()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(actions).encode())
        else:
            super().do_GET()

def start_server():
    rospy.init_node('web_dashboard_host_sawyer')
    port = rospy.get_param('~port', 8000)
    
    # Set the working directory to the www folder
    package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    www_path = os.path.join(package_path, 'www')
    
    if not os.path.exists(www_path):
        os.makedirs(www_path)
    
    os.chdir(www_path)
    
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", port), MyHandler) as httpd:
        rospy.loginfo("Sawyer Web Dashboard hosting at http://0.0.0.0:%d", port)
        rospy.loginfo("Serving files from: %s", www_path)
        httpd.serve_forever()

if __name__ == '__main__':
    try:
        start_server()
    except rospy.ROSInterruptException:
        pass
