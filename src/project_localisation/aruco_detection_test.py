
import cv2

# sample_img = (
#     "/home/metr4202/metr4202_2026_team30/"
#     "project_localisation/project_localisation/"
#     "aruco_camera_frame.png"
# )
sample_img = "/home/metr4202/metr4202_2026_team30/project_localisation/project_localisation/aruco_camera_frame.png"

image = cv2.imread(sample_img)

if image is None:
    raise FileNotFoundError(f"Could not load image: {sample_img}")


gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Save grayscale and enhanced-contrast versions
cv2.imwrite("aruco_grayscale.png", gray)

enhanced = cv2.equalizeHist(gray)
cv2.imwrite("aruco_enhanced.png", enhanced)

print("Saved aruco_grayscale.png")
print("Saved aruco_enhanced.png")
print("OpenCV version:", cv2.__version__)
print("Image dimensions:", image.shape[1], "x", image.shape[0])

dictionaries = {
    "6X6_50": cv2.aruco.DICT_6X6_50,
    "6X6_100": cv2.aruco.DICT_6X6_100,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "6X6_1000": cv2.aruco.DICT_6X6_1000,
}

for name, dictionary_id in dictionaries.items():
    dictionary = cv2.aruco.Dictionary_get(dictionary_id)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray, dictionary, parameters=parameters
    )

    if ids is not None and len(ids) > 0:
        print(f"{name}: detected IDs {ids.flatten().tolist()}")

        result = image.copy()
        cv2.aruco.drawDetectedMarkers(result, corners, ids)
        cv2.imwrite(f"aruco_result_{name}.png", result)
    else:
        print(f"{name}: no marker detected")

cv2.imwrite("aruco_input_check.png", image)
