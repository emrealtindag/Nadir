# Model Weights

This directory contains trained neural network weights for the Nadir precision landing system.

## Available Models

### `best.pt`
- **Architecture**: Ultralytics YOLOv8 Object Detection / Landability Model
- **Input Resolution**: 640x640 (dynamic)
- **Classes**: Dynamic ground obstacle detection (vehicles, pedestrians, landing targets, hazards)
- **Role in Pipeline**: Evaluates the landability score of the designated landing region in real time and highlights obstacles on the visual HUD during descent.

## Usage
The model weights are loaded automatically by `YOLOLandabilityPlugin` in `nadir.perception.landability.yolo_plugin`:
```python
from nadir.perception.landability.yolo_plugin import YOLOLandabilityPlugin

plugin = YOLOLandabilityPlugin(weights_path="weights/best.pt", confidence_threshold=0.35)
```
