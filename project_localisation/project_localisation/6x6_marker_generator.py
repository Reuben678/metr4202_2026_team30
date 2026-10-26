"""
Code for testing ArUco marker detection by generating a marker
and checking whether OpenCV can successfully detect it.
"""

import cv2
import random
import numpy as np


"""Display the installed OpenCV version."""
print("OpenCV version:", cv2.__version__)


# Generate marker from the predefined 6x6_250 ArUco dictionary.
dictionary = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_6X6_250
)

# Select random marker ID within dictionary's valid range.
marker_id = random.randint(0, 249)

# Render selected marker as 200x200px image.
marker = cv2.aruco.drawMarker(dictionary, marker_id, 200)


# Create white canvas with a margin around the marker.
image = np.full((260, 260), 255, dtype=np.uint8)

# Place marker in the center of the canvas.
image[30:230, 30:230] = marker

# Save generated image
cv2.imwrite("aruco_self_test.png", image)


# Initialize default ArUco marker detection parameters.
parameters = cv2.aruco.DetectorParameters_create()

# Detect ArUco markers in the generated image.
# Returns the detected corners, marker IDs, and rejected candidates.
corners, ids, rejected = cv2.aruco.detectMarkers(
    image,
    dictionary,
    parameters=parameters
)

# Report whether generated marker was successfully detected.
if ids is not None:
    print("SUCCESS: Detected marker IDs:", ids.flatten().tolist())
else:
    print("FAILED: OpenCV did not detect its own generated marker.")

# Display number of rejected marker candidates.
print("Rejected candidates:", len(rejected))

# Confirm output image's filename.
print("Saved test image: aruco_self_test.png")
