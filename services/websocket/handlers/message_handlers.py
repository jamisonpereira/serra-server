# websocket/handlers/message_handlers.py

from flask_socketio import emit, disconnect
from flask import request
from middleware.authenticate import verify_token
import json
from .test_handlers import handle_test_message

def handle_message(data):
    """
    Handle incoming WebSocket messages.
    """
    # Authenticate each message
    token = request.args.get('token')
    result = verify_token(token)

    if isinstance(result, tuple):  # Invalid token case
        emit('error', {'error': 'Unauthorized access'})
        disconnect(request.sid)
        return

    try:
        message_data = json.loads(data)
        message_type = message_data.get('type')

        if message_type == 'test':
            handle_test_message(message_data)
        elif message_type == 'landing_permission':
            handle_landing_permission(message_data)
        elif message_type == 'location_data':
            handle_location_data(message_data)
        elif message_type == 'return_to_base':
            handle_return_to_base(message_data)
        else:
            emit('error', {'error': 'Unsupported message type'}, to=request.sid)

    except json.JSONDecodeError:
        emit('error', {'error': 'Invalid JSON format'}, to=request.sid)

def handle_landing_permission(data):
    """
    Handle landing permission request.
    """
    print(f"Landing permission data received: {data}")
    emit('landing_permission_response', {'response': 'Landing permission processed'}, to=request.sid)

def handle_location_data(data):
    """
    Handle location data.
    """
    print(f"Location data received: {data}")
    emit('location_data_response', {'response': 'Location data processed'}, to=request.sid)

def handle_return_to_base(data):
    """
    Handle return to base request.
    """
    print(f"Return to base request received: {data}")
    emit('return_to_base_response', {'response': 'Return to base processed'}, to=request.sid)
