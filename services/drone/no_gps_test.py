import time
import sys
from dronekit import connect, VehicleMode
from pymavlink import mavutil

##### Variables ######

targetAltitude = 1
manualArm = False

##### Functions ######

def connect_to_drone():
    try:
        vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
        print("Drone connected successfully!")
        return vehicle
    except Exception as e:
        print("Failed to connect to the drone:", str(e))
        return None

def configure_no_gps(vehicle):
    # Disable GPS checks
    print("Disabling GPS-related arming checks...")
    vehicle.parameters['ARMING_CHECK'] = 0  # Disable all pre-arm checks
    vehicle.parameters['GPS_TYPE'] = 0  # Set GPS type to None

    # Wait for parameters to be set
    while vehicle.parameters['ARMING_CHECK'] != 0 or vehicle.parameters['GPS_TYPE'] != 0:
        print("Waiting for parameters to be updated...")
        time.sleep(1)

def reset_parameters(vehicle):
    # Reset GPS checks to defaults
    print("Resetting GPS-related arming checks to defaults...")
    vehicle.parameters['ARMING_CHECK'] = 1  # Re-enable all pre-arm checks
    vehicle.parameters['GPS_TYPE'] = 1  # Set GPS type to GPS module (usually 1 for u-blox GPS)

    # Wait for parameters to be reset
    while vehicle.parameters['ARMING_CHECK'] != 1 or vehicle.parameters['GPS_TYPE'] != 1:
        print("Waiting for parameters to be reset to defaults...")
        time.sleep(1)
    print("Parameters reset to defaults.")

def arm_and_takeoff(vehicle, targetHeight):
    # Configure vehicle to work without GPS
    configure_no_gps(vehicle)
    
    # Ensure vehicle is armable
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")
    
    # Set mode to GUIDED
    vehicle.mode = VehicleMode("GUIDED")
            
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")
 
    time.sleep(1)
 
    # Arm the vehicle
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
    
    # Takeoff to target altitude
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

###### Main ######

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

    # Reset parameters to default settings
    reset_parameters(vehicle)
    
    print('Drone takeoff and landing sequence complete')

if __name__ == "__main__":
    main()
