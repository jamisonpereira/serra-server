# websocket/handlers/drone_handlers.py

from flask_socketio import emit  # noqa: F401
from threading import Thread, Event
from services.drone.droneService import get_current_position, connect_drone

# Placeholder function to get the current drone coordinates

def broadcast_drone_coordinates(socketio, interval=5):
    """
    Broadcast the drone's current coordinates to all connected clients at a regular interval.
    """    
    print('Inside broadcast_drone_coordinates function.')
    stop_event = Event()
    print('stop_event: ', stop_event)
    vehicle = connect_drone()

    def broadcast():
        print('Inside broadcast function...')
        while not stop_event.is_set():
            print('Inside while loop')
            try:
                lat, long, alt = get_current_position(vehicle)
                coordinates = {
                    'lat': lat,
                    'long': long,
                    'alt': alt
                }
                socketio.emit('drone_coordinates', coordinates)
                print(f'Broadcasting drone coordinates: {coordinates}')
            except Exception as e:
                print(f'Error broadcasting drone coordinates: {e}')
                
            socketio.sleep(interval) 

    # Create a thread to broadcast coordinates
    print('Creating broadcast thread...')
    # broadcast_thread = Thread(target=broadcast)
    # broadcast_thread.daemon = True
    # broadcast_thread.start()
    socketio.start_background_task(broadcast)
    print('Broadcast thread started.')

    return stop_event
