# websocket/handlers/connection_handlers.py

from flask_socketio import disconnect
from flask import request
from middleware.authenticate import verify_token
from ..state import connected_clients  # Import connected_clients from state module
import threading
import os

connected_clients_lock = threading.Lock()

def handle_connect():
    token = request.args.get('token')
    drone_password = request.headers.get('Drone-Password')
    # result = verify_token(token)
    result = verify_token(token=token, drone_password=drone_password)

    # if isinstance(result, tuple):  # Invalid token case
    #     print('Invalid token. Disconnecting client.')
    #     disconnect()
    #     return False  # Disconnect the client

    if not result:  # Invalid token or drone password
        print('Invalid token or drone password. Disconnecting client.')
        disconnect()
        return False  # Disconnect the client

    # user = result  # Valid user
    # if not drone_password:
    with connected_clients_lock:
        connected_clients.add(request.sid)  # Track connection
    print('Connected websocket')
    # print(f"Authenticated user: {user}")
    print(f'connected_clients (after connect): {connected_clients}')

def handle_disconnect():
    with connected_clients_lock:
        connected_clients.discard(request.sid)
    print(f"Client {request.sid} disconnected")
    print(f'connected_clients (after disconnect): {connected_clients}')
