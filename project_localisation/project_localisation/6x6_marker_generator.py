
import cv2
import random
import numpy as np

print("OpenCV version:", cv2.__version__)

# Generate a known 6x6_250 marker
dictionary = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_6X6_250
)

marker_id = random.randint(0, 249)
marker = cv2.aruco.drawMarker(dictionary, marker_id, 200)

# Add a white margin around the marker
image = np.full((260, 260), 255, dtype=np.uint8)
image[30:230, 30:230] = marker

cv2.imwrite("aruco_self_test.png", image)

# Detect the marker
parameters = cv2.aruco.DetectorParameters_create()

corners, ids, rejected = cv2.aruco.detectMarkers(
    image,
    dictionary,
    parameters=parameters
)

if ids is not None:
    print("SUCCESS: Detected marker IDs:", ids.flatten().tolist())
else:
    print("FAILED: OpenCV did not detect its own generated marker.")

print("Rejected candidates:", len(rejected))
print("Saved test image: aruco_self_test.png")
