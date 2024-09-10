# controllers/resupplyController.py

from flask import request, jsonify
from dronekit import LocationGlobalRelative
from services.drone.droneService import connect_drone, set_vehicle_parameters, arm_and_takeoff, travel_to_gps_coordinate, travel_to_local_coordinate, convert_local_to_gps, get_current_position, calculate_distance
# from services.drone.precisionLandingService import precision_landing
import time
import subprocess
import os

def request_resupply():
  # Connect to the drone
  
  #TODO: add error handling to send back a response if the connection fails and other functions fail.
  connection_string = 'udp:127.0.0.1:14550'
  drone = connect_drone(connection_string=connection_string)

  parameters = {
    'PLND_ENABLED': 1,
    'PLND_TYPE': 1,
    'PLND_EST_TYPE': 0,
    'LAND_SPEED': 30
  } #TODO: make these parameters configurable in app

  # Set vehicle parameters
  set_vehicle_parameters(vehicle=drone, params=parameters)

  target_height = 4 #TODO: make this height configurable in app

  # Arm and takeoff
  arm_and_takeoff(vehicle=drone, target_height=target_height)

  # Get GPS coordinate from request
  # gps_coordinate = request.json.get('gps_coordinate')
  # latitude = gps_coordinate.get('latitude')
  # longitude = gps_coordinate.get('longitude')
  # altitude = gps_coordinate.get('altitude')
  
  # Travel to GPS coordinate
  # travel_to_gps_coordinate(vehicle=drone, latitude=latitude, longitude=longitude, altitude=altitude)

######### For testing in Gazebo #########
  # Set the target local coordinate in Gazebo for testing (7.5 meters east, 0 meters north)
  x = 0  # East in meters
  y = 7.5    # North in meters
  altitude = drone.location.global_relative_frame.alt  # Current altitude
  target_latitude, target_longitude = convert_local_to_gps(drone, x, y)
  # target_location = LocationGlobalRelative(target_latitude, target_longitude, altitude)
  
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

  if start_precision_landing.lower() == "y" or start_precision_landing.lower() == "yes":
      print('Executing precision landing.')

      # Run the wrapper scriptb
      # subprocess.Popen(['/usr/local/bin/run_precision_landing_wrapper.sh'])
      subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', '/usr/local/bin/run_precision_landing_wrapper.sh; exec bash'])


#########################################

  return jsonify({'message': 'Drone resupply processed.'})
