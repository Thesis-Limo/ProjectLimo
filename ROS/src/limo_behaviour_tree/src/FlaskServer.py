#!/usr/bin/env python3.6
import rospy
from std_msgs.msg import String
from flask import Flask, render_template
from flask_socketio import SocketIO


# ROS callback function to handle incoming log messages
def log_callback(msg):
    global log_data
    log_data = msg.data
    # Emit log data to all connected clients
    socketio.emit('log_update', log_data)


if __name__ == '__main__':
    app = Flask(__name__,template_folder='./')
    socketio = SocketIO(app)
    log_data = ""
    # ROS node initialization
    rospy.init_node('log_listener', anonymous=True)

    # Subscribe to the /BT/Log topic
    rospy.Subscriber("/BT/Log", String, log_callback)


    # Start Flask-SocketIO server
    socketio.run(app, debug=True, host='0.0.0.0',port=5001)

@app.route('/')
def index():
    return render_template('index.html')