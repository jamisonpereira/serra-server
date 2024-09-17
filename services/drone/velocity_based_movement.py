##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
import sys
from pymavlink import mavutil


##########VARIABLES#############

targetAltitude = 1.5
manualArm = False
boxSize = 2

#########FUNCTIONS#################

def connect_to_drone():
    try:
        vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
        print("Drone connected successfully!")
        return vehicle
    except Exception as e:
        print("Failed to connect to the drone:", str(e))
        return None

def arm_and_takeoff(vehicle, targetHeight):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")
    
    vehicle.mode = VehicleMode("GUIDED")
            
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")
 
    time.sleep(1)
 
    if not manualArm:
        vehicle.armed = True
        while not vehicle.armed:
            print("Waiting for vehicle to become armed.")
            time.sleep(1)
        print("Look out! Props are spinning!!")
    else:
        if not vehicle.armed:
            print("Exiting script. manualArm set to True but vehicle not armed.")
            print("Set manualArm to True if desiring script to arm the drone.")
            return
  
    time.sleep(1)
    vehicle.simple_takeoff(targetHeight)  # meters

    while True:
        print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= 0.95 * targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")
    
    return None

##Send a velocity command with +x being the heading of the drone.
def send_local_ned_velocity(vehicle, vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

##########MAIN EXECUTABLE###########

def main():
  print('Starting drone velocity-based movement sequence')
  vehicle = connect_to_drone()
  if vehicle is None:
      sys.exit(1)
  
  arm_and_takeoff(vehicle, targetAltitude)
  time.sleep(2)
  
  counter = 0
  while counter < boxSize:
    send_local_ned_velocity(vehicle, 1, 0, 0)
    time.sleep(1)
    print("Moving EAST relative to front of drone")
    counter += 1
  
  time.sleep(2)
  
  counter = 0
  while counter < boxSize:
    send_local_ned_velocity(vehicle, 0, 1, 0)
    time.sleep(1)
    print("Moving NORTH relative to front of drone")
    counter += 1
  
  time.sleep(2)
  
  counter = 0
  while counter < boxSize:
    send_local_ned_velocity(vehicle, -1, 0, 0)
    time.sleep(1)
    print("Moving WEST relative to front of drone")
    counter += 1
  
  time.sleep(2)
  
  counter = 0
  while counter < boxSize:
    send_local_ned_velocity(vehicle, 0, -1, 0)
    time.sleep(1)
    print("Moving SOUTH relative to front of drone")
    counter += 1
  
  time.sleep(2)
  
  vehicle.mode = VehicleMode("RTL")
  print("Returning to launch...")
  
  while vehicle.mode != 'RTL':
    print("Waiting for drone to enter RTL flight mode")
    time.sleep(1)
  
  print("Vehicle now in RTL MODE. Returning to launch...")
  
  return None

if __name__ == "__main__":
  main()
