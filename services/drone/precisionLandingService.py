# # services/drone/precisionLandingService.py

# from sensor_msgs.msg import Image
# import cv2
# import cv2.aruco as aruco
# import numpy as np
# import ros_numpy as rnp
# import threading
# import rospy
# import time
# import math
# from services.drone.droneService import send_land_message
# from dronekit import VehicleMode



# ############## Variables #############

# velocity = 1  # m/s
# takeoff_height = 3.5  # meters

# # ROS Publisher
# newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

# # ArUco marker settings
# id_to_find = 72
# marker_size = 40  # cm
# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# parameters = aruco.DetectorParameters_create()

# # Camera intrinsics
# horizontal_res = 640
# vertical_res = 480
# horizontal_fov = 62.2 * (math.pi / 180)
# vertical_fov = 48.8 * (math.pi / 180)

# dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
# camera_matrix = [[530.8269276712998, 0.0, 320.5], [0.0, 530.8269276712998, 240.5], [0.0, 0.0, 1.0]]
# np_dist_coeff = np.array(dist_coeff)
# np_camera_matrix = np.array(camera_matrix)

# # Time variables
# time_last = 0
# time_to_wait = 0.1  # 100 ms

# vehicle = None  # Initialize the vehicle variable

# marker_found = False  # Flag to indicate if the marker is found

# def detect_aruco_marker(image_data):
#     """Detects the specified ArUco marker in the given image data and handles landing logic."""
#     global marker_found, time_last, time_to_wait, id_to_find

#     np_data = rnp.numpify(image_data)  # Deserialize image data into array
#     gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
#     corners, ids, _ = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

#     if time.time() - time_last > time_to_wait:
#         if ids is not None:
#             # Filter to include only the desired marker ID
#             ids = ids.flatten()
#             index = np.where(ids == id_to_find)[0]
#             if len(index) > 0:  # Check if the desired marker ID is present
#                 marker_found = True
#                 corner_group = corners[index[0]]
#                 ret = aruco.estimatePoseSingleMarkers(corner_group, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
#                 rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
#                 x, y, z = '{:.2f}'.format(tvec[0]), '{:.2f}'.format(tvec[1]), '{:.2f}'.format(tvec[2])
#                 marker_position = 'MARKER POSITION x='+x+'y='+y+' z='+z

#                 # Calculate average position of the corners to determine the marker's center in image
#                 y_sum = np.sum(corner_group[0][:, 1])
#                 x_sum = np.sum(corner_group[0][:, 0])
#                 x_avg = x_sum / 4
#                 y_avg = y_sum / 4

#                 # Convert to angles relative to camera
#                 x_ang = (x_avg - horizontal_res * 0.5) * horizontal_fov / horizontal_res
#                 y_ang = (y_avg - vertical_res * 0.5) * vertical_fov / vertical_res

#                 # Send landing messages
#                 if vehicle.mode != 'LAND':
#                     vehicle.mode = VehicleMode('LAND')
#                     while vehicle.mode != 'LAND':
#                         time.sleep(1)
#                     print('Vehicle is in LAND mode')
#                 send_land_message(x_ang, y_ang)
         
#                 # Publish new image message for visualization
#                 new_msg = rnp.msgify(Image, np_data, encoding='bgr8')
#                 newimg_pub.publish(new_msg)

#                 # Draw the detected marker and its axis
#                 aruco.drawDetectedMarkers(np_data, corners)
#                 aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)
#                 cv2.putText(np_data, marker_position, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)

#                 # Show the image for debugging
#                 cv2.imshow("Detected Markers", np_data)
#                 cv2.waitKey(1)

#                 time_last = time.time()

#         return marker_found
#     else:  
#         return marker_found

# def start_computer_vision():
#     """Start the ROS subscriber to detect the ArUco marker."""
#     rospy.init_node('drone_node', anonymous=False)
#     rospy.Subscriber('/camera/color/image_raw', Image, detect_aruco_marker)
#     rospy.spin()  # Keep the subscriber running

# def precision_landing(drone):
#     global marker_found, vehicle
#     vehicle = drone
#     start_computer_vision()    
    

#     # # Create and start the subscriber thread
#     # subscriber_thread = threading.Thread(target=start_subscriber)
#     # subscriber_thread.start()

#     # # Wait for the subscriber thread to complete
#     # subscriber_thread.join()

