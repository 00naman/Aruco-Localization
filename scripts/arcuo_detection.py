import cv2
import cv2.aruco as aruco

# Load the ArUco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)

# Load the marker image from a file (replace 'aruco_marker.png' with your image file path)
marker_image = cv2.imread('/home/naman/sim_ws/src/m2wr_description/arucotest4.png', cv2.IMREAD_GRAYSCALE)

# Detect the ArUco marker in the image
corners, ids, _ = aruco.detectMarkers(marker_image, aruco_dict)

if ids is not None:
    # Draw the detected marker on the image
    aruco.drawDetectedMarkers(marker_image, corners, ids)

    # Print the detected marker's ID and corners
    print(f"Detected ArUco Marker ID: {ids[0][0]}")
    print(f"Corner Coordinates: {corners[0]}")

    # Display the image with the detected marker
    cv2.imshow('Detected ArUco Marker', marker_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No ArUco marker detected in the image.")
