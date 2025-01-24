import yaml

# Load map parameters from YAML file
yaml_path = "map_basement_new.yaml"

with open(yaml_path, "r") as file:
    map_params = yaml.safe_load(file)

# Extract required parameters
resolution = map_params["resolution"]  # meters per pixel
origin = map_params["origin"]  # [x, y, yaw]

import cv2
import numpy as np

# Load the PGM file
pgm_path = map_params["image"]
map_img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)

# Convert to binary (0 = occupied, 255 = free space)
binary_map = (map_img < 250).astype(np.uint8) * 255  # Thresholding

# Find contours of the enclosed space
contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Get bounding box of enclosed area
x, y, w, h = cv2.boundingRect(contours[0])

# Compute real-world dimensions
real_width = w * resolution
real_height = h * resolution

# Convert pixel coordinates to world coordinates
map_x_min = origin[0] + x * resolution
map_y_min = origin[1] + y * resolution
map_x_max = map_x_min + real_width
map_y_max = map_y_min + real_height

print(f"Enclosed area dimensions: {real_width:.2f}m x {real_height:.2f}m")
print(f"World coordinates: x_min={map_x_min:.2f}, y_min={map_y_min:.2f}, x_max={map_x_max:.2f}, y_max={map_y_max:.2f}")

# import matplotlib.pyplot as plt

# plt.figure(figsize=(10, 10))
# plt.imshow(cropped_map, cmap="gray")
# plt.title(f"Enclosed Room: {real_width:.2f}m x {real_height:.2f}m")
# plt.show()
