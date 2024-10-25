# websocket/handlers/test_handlers.py

from flask_socketio import emit
from flask import request

def handle_test_message(data):
    """
    Example function to handle 'test' WebSocket messages.
    """
    print(f"Test message received: {data}")
    emit('test_response', {'response': 'Test message received'}, to=request.sid)
