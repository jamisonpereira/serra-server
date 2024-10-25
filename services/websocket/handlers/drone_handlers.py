# websocket/handlers/drone_handlers.py

from flask_socketio import emit
from threading import Thread, Event
import time
from services.drone.droneService import get_current_position 

# Placeholder function to get the current drone coordinates


def broadcast_drone_coordinates(socketio, interval=5):
    """
    Broadcast the drone's current coordinates to all connected clients at a regular interval.
    """    
    stop_event = Event()

    def broadcast():
        while not stop_event.is_set():
            lat, long, alt = get_current_position()
            coordinates = {
                'lat': lat,
                'long': long,
                'alt': alt
            }
            socketio.emit('drone_coordinates', coordinates)
            time.sleep(interval)

    # Create a thread to broadcast coordinates
    broadcast_thread = Thread(target=broadcast)
    broadcast_thread.daemon = True
    broadcast_thread.start()

    return stop_event
