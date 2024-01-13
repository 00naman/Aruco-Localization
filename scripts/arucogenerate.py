import cv2
import numpy as np

# Define the dictionary and marker ID
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
marker_id = 1  # Change this to the desired marker ID (0 to 99)

# Generate the ArUco marker
marker_size = 200  # Adjust the marker size as needed
aruco_marker = cv2.aruco.drawMarker(dictionary, marker_id, marker_size)

# Rotate the ArUco marker image by 270 degrees counterclockwise (or 90 degrees clockwise)
rotated_aruco_marker = cv2.rotate(aruco_marker, cv2.ROTATE_90_COUNTERCLOCKWISE)

# Save the rotated ArUco marker as an image
output_file = f"aruco_{5}x{5}_100_{marker_id}_rotated.png"
cv2.imwrite(output_file, rotated_aruco_marker)

print(f"Rotated ArUco marker saved as {output_file}")