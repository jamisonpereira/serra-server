# websocket/handlers/drone_handlers.py

from flask_socketio import emit  # noqa: F401
from threading import Thread, Event
from services.drone.droneService import get_current_position, connect_drone, calculate_distance

# Placeholder function to get the current drone coordinates

def broadcast_drone_coordinates(socketio, interval=5):
    """
    Broadcast the drone's current coordinates to all connected clients at a regular interval.
    """    
    print('Inside broadcast_drone_coordinates function.')
    stop_event = Event()
    print('stop_event: ', stop_event)
    vehicle = connect_drone()
    previous_coordinates = None

    def broadcast():
        print('Inside broadcast function...')
        nonlocal previous_coordinates
        while not stop_event.is_set():
            print('Inside while loop')
            try:
                lat, long, alt = get_current_position(vehicle)
                current_coordinates = (lat, long, alt)
                
                if previous_coordinates:
                    distance = calculate_distance(
                        previous_coordinates[0], previous_coordinates[1], previous_coordinates[2],
                        current_coordinates[0], current_coordinates[1], current_coordinates[2]
                    )
                    if distance <= 0.25:
                        print(f'Coordinates have not changed significantly: {distance:.2f} meters')
                        socketio.sleep(interval)
                        continue

                coordinates = {
                    'lat': lat,
                    'long': long,
                    'alt': alt
                }
                socketio.emit('drone_coordinates', coordinates)
                print(f'Broadcasting drone coordinates: {coordinates}')
                previous_coordinates = current_coordinates
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
