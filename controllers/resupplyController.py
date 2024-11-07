# controllers/resupplyController.py

from flask import request, jsonify
from dronekit import LocationGlobalRelative
from services.drone.droneService import connect_drone, set_vehicle_parameters, arm_and_takeoff, travel_to_gps_coordinate, travel_to_local_coordinate, convert_local_to_gps, get_current_position, calculate_distance, control_gimbal, control_gimbal_pitch_yaw
# from services.drone.precisionLandingService import precision_landing
import time
import subprocess
import os
from services.websocket.handlers.emit_handlers import emit_landing_request, emit_mission_status, emit_drone_status  # Import the reusable emit functions
# from services.resupply_state import landing_permission_event, landing_permission_granted  # Import landing event and flag
from services.resupply_state import shared_resupply_state  # Import the shared resupply state
from extensions import socketio  # Import the shared socketio instance

def request_resupply():
  # global landing_permission_granted 
  
  # Connect to the drone
  
  #TODO: add error handling to send back a response if the connection fails and other functions fail.
  connection_string = 'udp:127.0.0.1:14550'
  drone = connect_drone(connection_string=connection_string)
  emit_mission_status(socketio, drone_id='DRONE123', status='OnMission')  # Emit mission status to all connected clients

  parameters = {
    'PLND_ENABLED': 1,
    'PLND_TYPE': 1,
    'PLND_EST_TYPE': 0,
    'LAND_SPEED': 30
  } #TODO: make these parameters configurable in app

  # Set vehicle parameters
  set_vehicle_parameters(vehicle=drone, params=parameters)
  control_gimbal_pitch_yaw(drone, pitch=90, yaw=0, follow_body_frame=True)

  target_height = 2.3 #TODO: make this height configurable in app

  # Arm and takeoff
  arm_and_takeoff(vehicle=drone, target_height=target_height)

  print('Starting video stream service.')
  video_stream_script = "/home/jamison/ardu_ws/src/serra/serra/scripts/video_stream_service.py"
  ngrok_server = "wss://rattler-helped-hawk.ngrok-free.app"
  topic = "/camera/image"
  subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'python3 {video_stream_script} --server {ngrok_server} --topic {topic}; exec bash'])

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
  y = 10    # North in meters
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
          emit_drone_status(socketio, drone_id='DRONE123', status='Flying')  # Emit drone status to all connected clients
          if current_distance < how_close_to_target:
              print(f"Drone has reached the destination at coordinates ({x}, {y}) with altitude {altitude}.")
              # Emit landing request to client via reusable function
              emit_drone_status(socketio, drone_id='DRONE123', status='Loitering')  # Emit drone status to all connected clients
              emit_landing_request(socketio, drone_id='DRONE123', latitude=current_x, longitude=current_y)
              break
          time.sleep(1)  # Wait for 1 second before checking again    


  # # Prompt the user to start precision landing
  # start_precision_landing = input("Do you want to start precision landing? (y/n): ")


  # if start_precision_landing.lower() == "y" or start_precision_landing.lower() == "yes":
  #     print('Executing precision landing.')

  #     # Run the wrapper scriptb
  #     # subprocess.Popen(['/usr/local/bin/run_precision_landing_wrapper.sh'])
  #     # subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', '/usr/local/bin/run_precision_landing_wrapper.sh; exec bash'])
  #     precision_landing_script = "/home/jamison/ardu_ws/src/serra/serra/scripts/serverPrecisionLanding.py"
  #     subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'python3 {precision_landing_script}; exec bash'])

 # Wait for landing permission response from the user
  print("Waiting for user permission to proceed with landing...")
  shared_resupply_state.landing_permission_event.wait()  # Wait until landing_permission_event is set by the callback

  print(f'landing_permission_granted (resupplyController.py): {shared_resupply_state.landing_permission_granted}')
    # Proceed based on user's response
  if shared_resupply_state.landing_permission_granted:
        print('Executing precision landing.')
        # Run the precision landing script
        emit_drone_status(socketio, drone_id='DRONE123', status='Landing')  # Emit drone status to all connected clients
        precision_landing_script = "/home/jamison/ardu_ws/src/serra/serra/scripts/serverPrecisionLanding.py"
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'python3 {precision_landing_script}; exec bash'])
  else:
        print('Landing was not authorized by the user. Aborting landing sequence.')


#########################################

  return jsonify({'message': 'Drone resupply processed.'})
