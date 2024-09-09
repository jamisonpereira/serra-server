# controllers/resupplyController.py

from flask import request, jsonify
from services.drone.droneService import connect_drone, set_vehicle_parameters, arm_and_takeoff, travel_to_gps_coordinate

def request_resupply():
  # Connect to the drone
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
  gps_coordinate = request.json.get('gps_coordinate')
  latitude = gps_coordinate.get('latitude')
  longitude = gps_coordinate.get('longitude')
  altitude = gps_coordinate.get('altitude')

  # Travel to GPS coordinate
  travel_to_gps_coordinate(vehicle=drone, latitude=latitude, longitude=longitude, altitude=altitude)

  return jsonify({'message': 'Drone resupply processed.'})
