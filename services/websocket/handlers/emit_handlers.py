# websocket/handlers/emit_handlers.py

from flask_socketio import emit # noqa: F401
from ..state import connected_clients

def emit_landing_request(socketio, drone_id, latitude, longitude):
    """
    Emit a landing request event to all connected clients.
    
    Args:
        socketio: The Socket.IO instance to use for emitting events.
        drone_id (str): The ID of the drone sending the request.
        latitude (float): The latitude of the drone's current position.
        longitude (float): The longitude of the drone's current position.
    """
    if connected_clients:  # Only emit if there are connected clients
        print(f'Emitting landing request to {len(connected_clients)} clients...')
        socketio.emit('landing_request', {
            'drone_id': drone_id,
            'message': 'Drone is ready to land, please authorize.',
            'latitude': latitude,
            'longitude': longitude
        })
    else:
        print('No connected clients to emit landing request.')
