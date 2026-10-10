import cv2
import numpy as np
from pathlib import Path

# Nadir uses DICT_4X4_100, target_id_allowlist: [1]
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
marker_size = 1024

# Draw the marker ID 1
img = np.zeros((marker_size, marker_size), dtype=np.uint8)
img = cv2.aruco.generateImageMarker(aruco_dict, 1, marker_size, img, 1)

# Save to simulation models directory
out_path = Path(__file__).resolve().parents[1] / "simulation" / "gazebo" / "models" / "aruco_marker.png"
out_path.parent.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(out_path), img)
print(f"ArUco marker image generated at: {out_path}")
