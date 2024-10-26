# websocket/handlers/connection_handlers.py

from flask_socketio import disconnect
from flask import request
from middleware.authenticate import verify_token
from ..state import connected_clients  # Import connected_clients from state module
import threading

connected_clients_lock = threading.Lock()

def handle_connect():
    token = request.args.get('token')
    result = verify_token(token)

    if isinstance(result, tuple):  # Invalid token case
        disconnect()
        return False  # Disconnect the client

    user = result  # Valid user
    with connected_clients_lock:
        connected_clients.add(request.sid)  # Track connection
    print('Connected websocket')
    print(f"Authenticated user: {user}")
    print(f'connected_clients (after connect): {connected_clients}')

def handle_disconnect():
    with connected_clients_lock:
        connected_clients.discard(request.sid)
    print(f"Client {request.sid} disconnected")
    print(f'connected_clients (after disconnect): {connected_clients}')
