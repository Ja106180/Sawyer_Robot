#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import http.server
import socketserver
import threading

class MyHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

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
