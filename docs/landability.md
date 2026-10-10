# Landability Analysis Guide

The landability analysis subsystem (`src/nadir/perception/landability`) assesses the physical safety of the touchdown zone prior to and during descent.

---

## Detection Approaches

### 1. Heuristic Estimator (`heuristics.py`)
- Analyzes surface texture variance (`texture_var_min`) to detect untextured, specular, or water surfaces.
- Tracks dense optical flow vectors (`motion_threshold`) to spot moving objects or turbulent ground disturbances.

### 2. Deep Learning & YOLOv8 Plugin (`yolo_plugin.py`)
- Deep neural network inference utilizing custom fine-tuned weights (`weights/best.pt`).
- Detects dynamic obstacles, vehicles, and humans in the vicinity of the landing zone.
- **Safety Interlock**: If a human or high-risk obstacle is detected inside the landing perimeter, the confidence score drops below `abort_landability`, causing the FSM to trigger an immediate `ABORT` or go-around.
