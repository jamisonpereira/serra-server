# services/droneService.py

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Connect to the vehicle
def connect_drone(connection_string='127.0.0.1:14550'):
  try:
    print("Connecting to vehicle...")
    vehicle = connect(connection_string, wait_ready=True)
    print("Connected to vehicle.")
    return vehicle
  except Exception as e:
    print("Failed to connect to vehicle:", str(e))
    return None

# Set vehicle parameters
def set_vehicle_parameters(vehicle, params):
  """
  Sets the parameters of the vehicle.

  Args:
    vehicle (Vehicle): The vehicle object.
    params (dict): A dictionary containing the parameter names and their corresponding values.

  Returns:
    None
  """
  try:
    print("Setting vehicle parameters...")
    for param_name, param_value in params.items():
      print(f"Setting parameter {param_name} to {param_value}")
      vehicle.parameters[param_name] = param_value
    print("Vehicle parameters set successfully.")
  except Exception as e:
    print("Failed to set vehicle parameters:", str(e))

# Arm the vehicle and take off to a specified height
def arm_and_takeoff(vehicle, target_height):
    try:
        print("Arming and taking off...")
        while not vehicle.is_armable:
            print('Waiting for vehicle to become armable...')
            time.sleep(1)
        print('Vehicle is now armable.')

        vehicle.mode = VehicleMode('GUIDED')

        while vehicle.mode != 'GUIDED':
            print('Waiting for drone to enter GUIDED flight mode...')
            time.sleep(1)
        print('Vehicle now in GUIDED mode.')

        vehicle.armed = True
        while not vehicle.armed:
            print('Waiting for vehicle to become armed...')
            time.sleep(1)
        print('Vehicle armed!')

        vehicle.simple_takeoff(target_height)

        while True:
            print('Current Altitude: {:.2f}'.format(vehicle.location.global_relative_frame.alt))
            if vehicle.location.global_relative_frame.alt >= 0.95 * target_height:
                print('Target altitude reached!')
                break
            time.sleep(1)
    except Exception as e:
        print("Error in arm_and_takeoff:", str(e))

# Travel to a specific GPS coordinate
def travel_to_gps_coordinate(vehicle, latitude, longitude, altitude):
  location = LocationGlobalRelative(latitude, longitude, altitude)
  vehicle.simple_goto(location)

  while True:
    current_location = vehicle.location.global_relative_frame
    distance = calculate_distance(current_location.lat, current_location.lon, latitude, longitude)
    print('Distance to target location: {:.2f} meters'.format(distance))
    if (abs(current_location.lat - latitude) < 0.00001 and
      abs(current_location.lon - longitude) < 0.00001 and
      abs(current_location.alt - altitude) < 0.5):
      print('Target GPS coordinate reached!')
      break
    time.sleep(2)

def calculate_distance(lat1, lon1, lat2, lon2):
  """
  Calculates the distance between two GPS coordinates using the haversine formula.

  Args:
    lat1 (float): Latitude of the first coordinate.
    lon1 (float): Longitude of the first coordinate.
    lat2 (float): Latitude of the second coordinate.
    lon2 (float): Longitude of the second coordinate.

  Returns:
    float: The distance between the two coordinates in meters.
  """
  # Haversine formula
  R = 6371e3  # Earth's radius in meters
  phi1 = math.radians(lat1)
  phi2 = math.radians(lat2)
  delta_phi = math.radians(lat2 - lat1)
  delta_lambda = math.radians(lon2 - lon1)

  a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

  distance = R * c
  return distance

# Send a movement command to the drone
def send_movement_message(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # Use velocity components only
        0, 0, 0, 
        vx, 
        vy, 
        vz, 
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Send a landing command to the drone
def send_land_message(vehicle, x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        x, 
        y, 
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    
####### Testing in Gazebo #########

# Function to convert local ENU coordinates to GPS coordinates
def convert_local_to_gps(vehicle, x, y):
    # Get current position as the reference point
    lat_ref = vehicle.location.global_relative_frame.lat
    lon_ref = vehicle.location.global_relative_frame.lon

    # Calculate new GPS coordinates
    # Latitude offset (d_lat) is zero since y = 0
    d_lat = 0

    # Longitude offset calculation for x meters east
    d_lon = x / (111139 * math.cos(math.radians(lat_ref)))

    # New GPS coordinates
    new_latitude = lat_ref + d_lat
    new_longitude = lon_ref + d_lon

    return new_latitude, new_longitude

# Function to travel to a specific local coordinate in Gazebo
def travel_to_local_coordinate(vehicle, x, y, altitude):
    # Convert local coordinates (x, y) to GPS coordinates
    target_latitude, target_longitude = convert_local_to_gps(vehicle, x, y)

    # Define the target location
    target_location = LocationGlobalRelative(target_latitude, target_longitude, altitude)
    
    # Command the drone to fly to the target location
    vehicle.simple_goto(target_location)

    # Monitor the distance to the target
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = calculate_distance(current_location.lat, current_location.lon, target_latitude, target_longitude)
        print('Distance to target location: {:.2f} meters'.format(distance))
        
        # Check if the target location is reached
        if (abs(current_location.lat - target_latitude) < 0.00001 and
            abs(current_location.lon - target_longitude) < 0.00001 and
            abs(current_location.alt - altitude) < 0.5):
            print('Target GPS coordinate reached!')
            break
        
        time.sleep(2)