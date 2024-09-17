import time
import os
import platform
import sys

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil


#####Variables######

targetAltitude = 1
manualArm = False

#####Functions######

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

def land(vehicle):
    vehicle.mode = VehicleMode("LAND")

    while vehicle.mode != 'LAND':
        time.sleep(1)
        print("Waiting for drone to land")
    print("Drone in land mode. Exiting script.")


######Main######

def main():
    print('Starting drone takeoff and landing sequence')
    
    vehicle = connect_to_drone()
    if vehicle is None:
        sys.exit(1)
    
    print('Starting arm and takeoff sequence')
    arm_and_takeoff(vehicle, targetAltitude)
    
    print('Hovering for 5 seconds')
    time.sleep(5)
    
    print('Starting landing sequence')
    land(vehicle)
    
    print('Drone takeoff and landing sequence complete')

if __name__ == "__main__":
    main()