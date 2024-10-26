# services/drone/droneService.py

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

vehicle = None

# Connect to the vehicle
def connect_drone(connection_string='127.0.0.1:14550'):
    """
    Connect to the drone and return the vehicle object.
    This function uses a singleton pattern to ensure only one connection is made.
    """
    global vehicle
    if vehicle is None:
        try:
            print("Connecting to vehicle...")
            vehicle = connect(connection_string, wait_ready=True)
            print("Connected to vehicle.")
        except Exception as e:
            print("Failed to connect to vehicle:", str(e))
            return None
    else:
        print("Reusing existing vehicle connection.")
    return vehicle

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
            if vehicle.location.global_relative_frame.alt >= 0.85 * target_height:
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

def calculate_distance(lat1, lon1, alt1, lat2, lon2, alt2):
  """
  Calculates the distance between two GPS coordinates using the haversine formula.

  Args:
    lat1 (float): Latitude of the first coordinate.
    lon1 (float): Longitude of the first coordinate.
    alt1 (float): Altitude of the first coordinate.
    lat2 (float): Latitude of the second coordinate.
    lon2 (float): Longitude of the second coordinate.
    alt2 (float): Altitude of the second coordinate.

  Returns:
    float: The distance between the two coordinates in meters.
  """
  # Haversine formula
  R = 6371e3  # Earth's radius in meters
  phi1 = math.radians(lat1)
  phi2 = math.radians(lat2)
  delta_phi = math.radians(lat2 - lat1)
  delta_lambda = math.radians(lon2 - lon1)
  delta_alt = alt2 - alt1

  a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

  distance_2d = R * c
  distance_3d = math.sqrt(distance_2d ** 2 + delta_alt ** 2)
  return distance_3d

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

# Function to get the drone's current position
def get_current_position(vehicle):
    """
    Retrieve the current position of the drone.

    Args:
        vehicle: An object representing the drone, which contains location information.

    Returns:
        tuple: A tuple containing the latitude (x), longitude (y), and altitude of the drone.

    Example:
        lat, lon, alt = get_current_position(vehicle)
    """
    return vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt


####### Testing in Gazebo #########

# Function to convert local ENU coordinates to GPS coordinates
def convert_local_to_gps(vehicle, x, y):
    """
    Convert local coordinates (x, y) to GPS coordinates (latitude, longitude).

    This function takes a vehicle's current GPS position as a reference point and converts
    local x, y coordinates (in meters) to new GPS coordinates.

    Args:
        vehicle: An object representing the vehicle, which contains the current GPS position.
        x (float): The local x-coordinate (east-west direction) in meters.
        y (float): The local y-coordinate (north-south direction) in meters.

    Returns:
        tuple: A tuple containing the new latitude and longitude as floats.

    Notes:
        - The conversion assumes a flat Earth approximation, which is generally accurate for small distances.
        - The latitude offset is calculated by dividing the y-coordinate by the approximate number of meters per degree of latitude (111,139 meters).
        - The longitude offset is calculated by dividing the x-coordinate by the number of meters per degree of longitude, adjusted by the cosine of the reference latitude.
    """
    # Get current position as the reference point
    lat_ref = vehicle.location.global_relative_frame.lat
    lon_ref = vehicle.location.global_relative_frame.lon

    # Calculate latitude offset (north-south direction)
    d_lat = y / 111139  # Convert meters to degrees (approximately 111,139 meters per degree of latitude)

    # Calculate longitude offset (east-west direction)
    d_lon = x / (111139 * math.cos(math.radians(lat_ref)))  # Convert meters to degrees (adjusted by latitude)

    # New GPS coordinates
    new_latitude = lat_ref + d_lat
    new_longitude = lon_ref + d_lon

    return new_latitude, new_longitude

# Function to travel to a specific local coordinate in Gazebo
def travel_to_local_coordinate(vehicle, target_latitude, target_longitude, altitude):
    """
    Commands the drone to travel to a specific local coordinate.

    Args:
        vehicle (Vehicle): The connected vehicle object.
        target_latitude (float): The target latitude in degrees.
        target_longitude (float): The target longitude in degrees.
        altitude (float): The target altitude in meters.

    Returns:
        bool: True if the command is successfully sent.

    Example:
        travel_to_local_coordinate(vehicle, 37.7749, -122.4194, 10)
    """

    # Define the target location
    target_location = LocationGlobalRelative(target_latitude, target_longitude, altitude)
    
    # Command the drone to fly to the target location
    vehicle.simple_goto(target_location)
    
    return True

    # Monitor the distance to the target
    # while True:
    #     current_location = vehicle.location.global_relative_frame
    #     distance = calculate_distance(current_location.lat, current_location.lon, target_latitude, target_longitude)
    #     print('Distance to target location: {:.2f} meters'.format(distance))
        
    #     # Check if the target location is reached
    #     if (abs(current_location.lat - target_latitude) < 0.00001 and
    #         abs(current_location.lon - target_longitude) < 0.00001 and
    #         abs(current_location.alt - altitude) < 0.5):
    #         print('Target GPS coordinate reached!')
    #         break
        
    #     time.sleep(2)
        

########### GIMBAL CONTROLS ############

# Map common commands to RC channel and PWM values
gimbal_command_map = {
    'tilt': {
        'up': 1100,
        'down': 1900,
        'neutral': 1500
    },
    'roll': {
        'up': 1100,
        'down': 1900,
        'neutral': 1500
    }
}

# Gimbal channel mapping
gimbal_channel_map = {
    'tilt': 10,  # Channel 10 is for tilt
    'roll': 9    # Channel 9 is for roll
}


def control_gimbal(vehicle, command_type, command_direction):
    """
    Controls the gimbal by sending a MAV_CMD_DO_SET_SERVO command to adjust tilt or roll.

    Args:
        vehicle (Vehicle): The connected vehicle object.
        command_type (str): The type of command ('tilt' or 'roll').
        command_direction (str): The direction ('up', 'down', or 'neutral').

    Returns:
        None
    """
    try:
        # Get the correct channel and PWM value based on the command type and direction
        channel = gimbal_channel_map[command_type] + 1  # Channels are 0-indexed
        pwm_value = gimbal_command_map[command_type][command_direction]

        # Send MAV_CMD_DO_SET_SERVO command
        print(f"Setting {command_type} to {command_direction} (PWM: {pwm_value}) on channel {channel}")
        vehicle._master.mav.command_long_send(
            vehicle._master.target_system,    # target_system
            vehicle._master.target_component, # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command ID for setting servos
            0,                                # Confirmation
            channel,                          # Servo channel
            pwm_value,                        # PWM value (1000-2000)
            0, 0, 0, 0, 0                    # Unused parameters
        )
        print(f"Gimbal {command_type} set to {command_direction} (PWM: {pwm_value}) on channel {channel}")
    except KeyError:
        print(f"Invalid command type '{command_type}' or direction '{command_direction}'")
    except Exception as e:
        print(f"Error controlling gimbal: {e}")
        
def control_gimbal_pitch_yaw(vehicle, pitch=90, yaw=0, follow_body_frame=False):
    """
    Controls the gimbal's pitch and yaw using MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW to point the gimbal straight down.

    Args:
        vehicle (Vehicle): The connected vehicle object.
        pitch (float): Pitch angle in degrees (-90 for straight down).
        yaw (float): Yaw angle in degrees (0 is forward).
        follow_body_frame (bool): If True, gimbal yaw follows the vehicle body frame; if False, yaw locks to the earth frame.

    Returns:
        None
    """
    print(f"Setting gimbal pitch={pitch}, yaw={yaw}, follow_body_frame={follow_body_frame}")
    try:
        # Define the flags for yaw control (0 = follow body frame, 16 = lock to earth frame)
        yaw_control_flag = 0 if follow_body_frame else 16

        # Send MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW command
        vehicle._master.mav.command_long_send(
            vehicle._master.target_system,    # Target system ID
            vehicle._master.target_component, # Target component ID
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,  # Command ID
            0,                                # Confirmation
            pitch,                            # Pitch angle in degrees (-90 for straight down)
            yaw,                              # Yaw angle in degrees (0 = forward)
            float('nan'),                     # Pitch rate (not used)
            float('nan'),                     # Yaw rate (not used)
            yaw_control_flag,                 # 0=Follow body frame, 16=Lock to earth frame
            0,                                # Not used
            0                                 # Gimbal device ID (0 = primary gimbal)
        )
        print(f"Setting gimbal (Pitch={pitch}, Yaw={yaw})")
    except Exception as e:
        print(f"Error setting gimbal pitch and yaw: {e}")