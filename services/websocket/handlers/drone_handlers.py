# websocket/handlers/drone_handlers.py

from flask_socketio import emit  # noqa: F401
from threading import Thread, Event
from services.drone.droneService import get_current_position, connect_drone, calculate_distance, convert_local_to_gps, get_gps_position_mavlink

# Placeholder function to get the current drone coordinates

def broadcast_drone_coordinates(socketio, interval=5):
    """
    Broadcast the drone's current coordinates to all connected clients at a regular interval.
    """    
    # print('Inside broadcast_drone_coordinates function.')
    stop_event = Event()
    # print('stop_event: ', stop_event)
    drone = connect_drone()
    previous_coordinates = None
    base_coordinates = {'lat': -35.3632621, 'long': 149.1652374, 'alt': -0.079}
    
    x = 0  # East in meters
    y = 10    # North in meters
    # my_altitude = drone.location.global_relative_frame.alt  # Current altitude
    # my_latitude, my_longitude = convert_local_to_gps(drone, x, y)
    # my_coordinates = {'lat': my_latitude, 'long': my_longitude, 'alt': my_altitude}
    my_latitude = -35.36317212258433
    my_longitude = 149.1652374
    my_altitude = -0.084
    my_coordinates = {'lat': my_latitude, 'long': my_longitude, 'alt': my_altitude}
    socketio.emit('my_coordinates', my_coordinates)

    def broadcast():
        # print('Inside broadcast function...')
        socketio.emit('base_coordinates', base_coordinates)
        print('Broadcasting base coordinates: ', base_coordinates)
        # socketio.emit('my_coordinates', my_coordinates)
        # print('Broadcasting my coordinates: ', my_coordinates)
        nonlocal previous_coordinates
        while not stop_event.is_set():
            # print('Inside while loop')
            try:
                lat, long, alt = get_current_position(drone)
                # lat, long, alt = get_gps_position_mavlink(drone)
                # print('***TEST****: Current drone coordinates: ', lat, long, alt)
                current_coordinates = (lat, long, alt)
                
                # my_altitude = drone.location.global_relative_frame.alt  # Current altitude
                # my_latitude, my_longitude = convert_local_to_gps(drone, x, y)
                # my_coordinates = {'lat': my_latitude, 'long': my_longitude, 'alt': my_altitude}
                # print('Broadcasting my coordinates: ', my_coordinates)
                speed = drone.groundspeed
                # battery = drone.battery.level
                battery = 100
                socketio.emit('speed', speed)
                socketio.emit('battery', battery)
                print(' Drone speed: ', speed)
                print(' Drone battery: ', battery)
                
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
                distance_me_to_drone = calculate_distance(my_latitude, my_longitude, my_altitude, lat, long, alt)
                socketio.emit('distance_me_to_drone', distance_me_to_drone)
                print('Broadcasting distance_me_to_drone: ', distance_me_to_drone)
                previous_coordinates = current_coordinates
            except Exception as e:
                print(f'Error broadcasting drone coordinates: {e}')
                
            socketio.sleep(interval) 

    # Create a thread to broadcast coordinates
    # print('Creating broadcast thread...')
    socketio.start_background_task(broadcast)
    # print('Broadcast thread started.')

    return stop_event
