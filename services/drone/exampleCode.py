#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import threading 
import signal

############## Variables #############

velocity = 1  # m/s
takeoff_height = 3.5  # meters

# ROS Publisher
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

# ArUco marker settings
id_to_find = 72
marker_size = 40  # cm
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# Camera intrinsics
horizontal_res = 640
vertical_res = 480
horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5], [0.0, 530.8269276712998, 240.5], [0.0, 0.0, 1.0]]
np_dist_coeff = np.array(dist_coeff)
np_camera_matrix = np.array(camera_matrix)

# Time variables
time_last = 0
time_to_wait = 0.1  # 100 ms

vehicle = None  # Initialize the vehicle variable

marker_found = False  # Flag to indicate if the marker is found

user_prompted = False
user_confirmed_landing = False

############## Position Filter Class #############
class PositionFilter:
    def __init__(self, window_size=5):
        self.values = []
        self.window_size = window_size

    def update(self, new_value):
        if len(self.values) >= self.window_size:
            self.values.pop(0)
        self.values.append(new_value)
        return sum(self.values) / len(self.values)

# Initialize position filters for x and y coordinates
x_filter = PositionFilter()
y_filter = PositionFilter()


############## FUNCTIONS ####################

def signal_handler(sig, frame):
    print('Signal caught, shutting down...')
    rospy.signal_shutdown('Interrupt received')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def setup_vehicle():
    """Connects to the vehicle and sets up parameters for precision landing."""
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    print("Connected to vehicle.")
    
    print("Setting vehicle parameters for precision landing...")
    vehicle.parameters['PLND_ENABLED'] = 1
    vehicle.parameters['PLND_TYPE'] = 1  # 1 = companion computer
    vehicle.parameters['PLND_EST_TYPE'] = 0  # 0 = ROS sensor data only
    vehicle.parameters['LAND_SPEED'] = 30  # cm/s
    print("Vehicle parameters set.")
    
    return vehicle

def arm_and_takeoff(targetHeight):
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

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('Current Altitude: {:.2f}'.format(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= 0.95 * targetHeight:
            print('Target altitude reached!')
            break
        time.sleep(1)

def send_movement_message(vx, vy, vz):
    # print("Sending movement command to drone...")
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
    # print("Movement command sent.")

def send_land_message(x, y):
    # print("Sending landing target coordinates to drone...")
    msg = vehicle.message_factory.landing_target_encode(
        0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        x, 
        y, 
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    # print("Landing target coordinates sent.")

MAX_ANGLE_CHANGE = 0.1  # Maximum allowed angle change per iteration (in radians)

def clamp(value, min_value, max_value):
    """Clamp the value between min_value and max_value."""
    return max(min_value, min(value, max_value))

def detect_aruco_marker(image_data):
    """Detects the specified ArUco marker in the given image data and handles landing logic."""
    global marker_found, time_last, time_to_wait, id_to_find, user_prompted, user_confirmed_landing, MAX_ANGLE_CHANGE

    np_data = rnp.numpify(image_data)  # Deserialize image data into array
    gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

    if time.time() - time_last > time_to_wait:
        if ids is not None:
            # Filter to include only the desired marker ID
            ids = ids.flatten()
            index = np.where(ids == id_to_find)[0]
            if len(index) > 0:  # Check if the desired marker ID is present
                marker_found = True
                corner_group = corners[index[0]]
                ret = aruco.estimatePoseSingleMarkers(corner_group, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
                x, y, z = '{:.2f}'.format(tvec[0]), '{:.2f}'.format(tvec[1]), '{:.2f}'.format(tvec[2])
                marker_position = 'MARKER POSITION x='+x+'y='+y+' z='+z

                # Prompt the user only once for landing confirmation
                if not user_prompted:
                    print()
                    print('Marker detected!')
                    user_input = raw_input("Commence landing? (yes/no): ")
                    user_prompted = True  # Set the flag to True after prompting
                    if user_input.lower() == 'yes' or 'y':
                        user_confirmed_landing = True  # Set the flag if the user confirms landing
                        print()
                        print("Commencing landing sequence...")
                        print()

                # Proceed with landing if the user has confirmed
                if user_confirmed_landing:
                    # Calculate average position of the corners to determine the marker's center in image
                    y_sum = np.sum(corner_group[0][:, 1])
                    x_sum = np.sum(corner_group[0][:, 0])
                    x_avg = x_sum / 4
                    y_avg = y_sum / 4

                    # Convert to angles relative to camera
                    x_ang = (x_avg - horizontal_res * 0.5) * horizontal_fov / horizontal_res
                    y_ang = (y_avg - vertical_res * 0.5) * vertical_fov / vertical_res

                    # Send landing messages
                    if vehicle.mode != 'LAND':
                        vehicle.mode = VehicleMode('LAND')
                        while vehicle.mode != 'LAND':
                            time.sleep(1)
                        # print('Vehicle is in LAND mode')
                    send_land_message(x_ang, y_ang)
                else:
                    print("User declined landing. Resuming operations.")
                    # If the user declines, you may want to add any logic to continue or hold the drone.

                # Publish new image message for visualization
                new_msg = rnp.msgify(Image, np_data, encoding='bgr8')
                newimg_pub.publish(new_msg)
                time_last = time.time()

                # Draw the detected marker and its axis
                aruco.drawDetectedMarkers(np_data, corners)
                aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)
                cv2.putText(np_data, marker_position, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)

                # Show the image for debugging
                cv2.imshow("Detected Markers", np_data)
                cv2.waitKey(1)

        return marker_found
    else:  
        return marker_found


def move_with_velocity(vx, vy, vz, duration):
    """Move the drone with velocity commands."""
    global marker_found

    # print("Starting movement with velocity (vx={}, vy={}, vz={}) for {} seconds...".format(vx, vy, vz, duration))
    start_time = time.time()

    while not marker_found and time.time() - start_time < duration:
        send_movement_message(vx, vy, vz)
        time.sleep(1)  # Sleep for 1 second

    # Stop the drone if the marker is found
    if marker_found:
        # print("Marker found.")
        send_movement_message(0, 0, 0)
        return

def start_subscriber():
    """Start the ROS subscriber to detect the ArUco marker."""
    # print("Starting ROS subscriber for ArUco marker detection...")
    # rospy.init_node('drone_node', anonymous=False)
    rospy.Subscriber('/camera/color/image_raw', Image, detect_aruco_marker)
    rospy.spin()  # Keep the subscriber running

def main():
    """Main function to run both tasks concurrently."""
    print('Beginning precision landing test...')

    # Vehicle setup
    global vehicle
    vehicle = setup_vehicle()  # Call the setup function and assign the vehicle

    # Perform the flight test
    arm_and_takeoff(takeoff_height)
    time.sleep(2)

    # Initialize the ROS node in the main thread
    rospy.init_node('drone_node', anonymous=False)

    # Create threads for concurrent execution
    movement_thread = threading.Thread(target=move_with_velocity, args=(velocity, 0, 0, 25))
    subscriber_thread = threading.Thread(target=start_subscriber)

    # Start both threads
    movement_thread.start()
    print()
    print('Moving out towards objective at 1 meter/second')
    print()
    print('Searching for landing marker...')
    subscriber_thread.start()

    # Wait for both threads to complete
    movement_thread.join()
    subscriber_thread.join()

if __name__ == '__main__':
    try:
        main()  # Run the main function
    except rospy.ROSInterruptException:
        print("ROS Interrupt Exception occurred. Exiting...")
        pass
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Exiting...")
        pass