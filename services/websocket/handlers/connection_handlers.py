# websocket/handlers/connection_handlers.py

from flask_socketio import disconnect
from flask import request
from middleware.authenticate import verify_token
from ..state import connected_clients  # Import connected_clients from state module

def handle_connect():
    token = request.args.get('token')
    result = verify_token(token)

    if isinstance(result, tuple):  # Invalid token case
        disconnect()
        return False  # Disconnect the client

    user = result  # Valid user
    connected_clients.add(request.sid)  # Track connection
    print('Connected websocket')
    print(f"Authenticated user: {user}")

def handle_disconnect():
    connected_clients.discard(request.sid)
    print(f"Client {request.sid} disconnected")