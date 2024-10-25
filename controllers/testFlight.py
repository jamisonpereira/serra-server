#!/usr/bin/env python3

import time
import subprocess
from dronekit import LocationGlobalRelative
from services.drone.droneService import connect_drone, set_vehicle_parameters, arm_and_takeoff, travel_to_gps_coordinate, travel_to_local_coordinate, convert_local_to_gps, get_current_position, calculate_distance, control_gimbal, control_gimbal_pitch_yaw
# from services.drone.precisionLandingService import precision_landing

def request_resupply():
    # Connect to the drone
    connection_string = 'udp:127.0.0.1:14550'
    drone = connect_drone(connection_string=connection_string)

    parameters = {
        'PLND_ENABLED': 1,
        'PLND_TYPE': 1,
        'PLND_EST_TYPE': 0,
        'LAND_SPEED': 30
    }  # TODO: make these parameters configurable in app

    # Set vehicle parameters
    set_vehicle_parameters(vehicle=drone, params=parameters)
    control_gimbal_pitch_yaw(drone, pitch=90, yaw=0, follow_body_frame=True)

    target_height = 2.3  # TODO: make this height configurable in app

    # Arm and takeoff
    arm_and_takeoff(vehicle=drone, target_height=target_height)

    ######### For testing in Gazebo #########
    # Set the target local coordinate in Gazebo for testing (7.5 meters east, 0 meters north)
    x = 0  # East in meters
    y = 10    # North in meters
    altitude = drone.location.global_relative_frame.alt  # Current altitude
    target_latitude, target_longitude = convert_local_to_gps(drone, x, y)

    how_close_to_target = 1  # How close the drone needs to be to the target in meters  

    # Travel to local coordinate
    print('Traveling to coordinates')
    if travel_to_local_coordinate(vehicle=drone, target_latitude=target_latitude, target_longitude=target_longitude, altitude=altitude):
        # Continuously check if the drone has reached the destination
        while True:
            current_x, current_y, current_altitude = get_current_position(vehicle=drone)
            current_distance = calculate_distance(current_x, current_y, current_altitude, target_latitude, target_longitude, altitude)
            print(f'Current distance to target: {current_distance}')
            if current_distance < how_close_to_target:
                print(f"Drone has reached the destination at coordinates ({x}, {y}) with altitude {altitude}.")
                break
            time.sleep(1)  # Wait for 1 second before checking again    

    # Prompt the user to start precision landing
    start_precision_landing = input("Do you want to start precision landing? (y/n): ")

    if start_precision_landing.lower() in ["y", "yes"]:
        print('Executing precision landing.')
        # subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', '/usr/local/bin/run_precision_landing_wrapper.sh; exec bash'])
        precision_landing_script = "/home/jamison/ardu_ws/src/serra/serra/scripts/serverPrecisionLanding.py"
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'python3 {precision_landing_script}; exec bash'])

#########################################

def main():
    print('Starting drone resupply request')
    request_resupply()
    print('Drone resupply sequence complete')

if __name__ == "__main__":
    main()
