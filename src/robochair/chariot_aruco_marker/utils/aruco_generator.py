import cv2
import numpy as np

# Define the dictionary and marker parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
marker_id = 0
marker_size = 50  # in pixels

# Generate the marker correctly using drawMarker
marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size, marker_image, 1)

# Save the marker image
cv2.imwrite("aruco_marker.png", marker_image)

# Display the generated marker
cv2.imshow("Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
