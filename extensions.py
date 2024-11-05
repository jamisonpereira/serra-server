# extensions.py

from flask_socketio import SocketIO

# Create SocketIO instance (not attached to app yet)
socketio = SocketIO(async_mode='threading')
