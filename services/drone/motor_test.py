from dronekit import connect, VehicleMode
import time
import sys
from pymavlink import mavutil

def connect_to_drone():
    print("Connecting to the drone...")
    try:
        # Adjust the connection string as needed
        vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
        print("Drone connected successfully!")
        return vehicle
    except Exception as e:
        print(f"Failed to connect to the drone: {e}")
        return None

def configure_no_gps(vehicle):
    print("Configuring vehicle to bypass GPS checks...")
    # Disable GPS checks
    vehicle.parameters['ARMING_CHECK'] = 0  # Disable all pre-arm checks
    vehicle.parameters['GPS_TYPE'] = 0      # Disable GPS
    time.sleep(1)  # Wait for parameters to be set
    print("GPS checks disabled.")

def run_motor_test(vehicle, throttle_percent=30, duration=5):
    print(f"Starting motor test: All motors at {throttle_percent}% throttle for {duration} seconds.")

    # Create the MAVLink command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, # command
        0,       # confirmation
        0,       # param1: Motor number (0 for all motors)
        1,       # param2: Test action (1: Motor test throttle)
        0,       # param3: Throttle type (0: percent)
        throttle_percent, # param4: Throttle (0-100%)
        duration, # param5: Timeout in seconds
        0, 0      # param6 and param7: unused
    )

    # Send the command to the vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print("Motor test initiated.")
    time.sleep(duration + 1)  # Wait for the test to complete

    # Optionally, stop the motor test (usually stops automatically after duration)
    msg_stop = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, # command
        0,       # confirmation
        0,       # param1: Motor number (0 for all motors)
        0,       # param2: Test action (0: Stop motor test)
        0, 0, 0, 0, 0  # Remaining params unused
    )
    vehicle.send_mavlink(msg_stop)
    vehicle.flush()
    print("Motor test concluded.")


def reset_parameters(vehicle):
    print("Resetting parameters to defaults...")
    vehicle.parameters['ARMING_CHECK'] = 1  # Re-enable pre-arm checks
    vehicle.parameters['GPS_TYPE'] = 1      # Re-enable GPS
    time.sleep(1)  # Wait for parameters to be set
    print("Parameters reset to defaults.")

def main():
    vehicle = connect_to_drone()
    if vehicle is None:
        sys.exit(1)

    configure_no_gps(vehicle)
    run_motor_test(vehicle, throttle_percent=30, duration=5)
    reset_parameters(vehicle)

    vehicle.close()
    print("Motor test complete. Vehicle connection closed.")

if __name__ == "__main__":
    main()
