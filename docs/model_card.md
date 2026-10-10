# YOLOv8 Landability Model Card

This model card documents the neural network architecture, dataset provenance, training protocol, and performance metrics for the custom object detector (`weights/best.pt`) deployed within the Nadir precision landing subsystem.

---

## 1. Model Overview

| Parameter | Specification |
|:---|:---|
| **Model Architecture** | YOLOv8n (Nano) |
| **Parameters** | 3.2 Million (Float32) |
| **Weights Artifact** | `weights/best.pt` |
| **Input Resolution** | $640 \times 640 \times 3$ (RGB) |
| **Primary Task** | Real-time aerial obstacle detection & spatial clearance gating |
| **Deployment Runtime** | PyTorch / ONNX Runtime / TensorRT |

---

## 2. Target Classes & Operational Roles

| Class ID | Native Label | English Mapping | Operational Role in Nadir FSM |
|:---:|:---:|:---:|:---|
| **0** | `Tasit` | Vehicle / Ground Obstacle | Collision hazard. If box intersects landing pad ROI: sets `score = 0.20`, raises `vehicle_present` $\rightarrow$ FSM `ABORT`. |
| **1** | `Insan` | Human / Personnel | Strict zero-tolerance personnel hazard. If box intersects landing pad ROI: sets `score = 0.00`, raises `human_present` $\rightarrow$ instant FSM `ABORT`. |

*Note: Classes outside `0` and `1` are filtered out at the inference layer to prevent spurious false positives.*

---

## 3. Dataset & Annotation Protocol

- **Dataset Size**: 2,450 high-resolution aerial and downward-facing UAV camera images.
- **Split Ratio**: 80% Training (1,960 images), 20% Validation (490 images).
- **Domain Coverage**:
  - Downward ($90^\circ$ nadir) and oblique ($45^\circ$) camera angles.
  - Multi-scale targets (ground vehicles from $10\text{ m}$ to $1\text{ m}$ AGL, personnel footprints).
  - Variable lighting conditions: overcast, direct sunlight, runway glare, and shadowed tarmac.
- **Annotation Process**:
  - Bounding boxes labeled in YOLO format (`class x_center y_center width height`).
  - Strict 3-pixel tight boundary margins to ensure accurate intersection calculations.
- **Data Augmentation**:
  - Albumentations pipeline: Mosaic augmentation ($p=1.0$), random HSV color jitter ($\pm 15\%$), horizontal flip ($p=0.5$), and perspective translation.

---

## 4. Training Hyperparameters

```yaml
epochs: 100
batch_size: 16
image_size: 640
optimizer: AdamW
learning_rate_initial: 0.001
learning_rate_final: 0.00001
lr_scheduler: cosine
warmup_epochs: 3.0
weight_decay: 0.0005
loss_functions:
  box_loss: Complete IoU (CIoU)
  cls_loss: Binary Cross-Entropy (BCE)
  dfl_loss: Distribution Focal Loss
```

---

## 5. Quantitative Evaluation Metrics

Evaluated on the held-out validation split (490 images) at IoU threshold $0.50$:

| Class | Precision | Recall | $\text{mAP}@0.50$ | $\text{mAP}@0.50:0.95$ |
|:---|:---:|:---:|:---:|:---:|
| **All Classes** | **0.918** | **0.886** | **0.924** | **0.741** |
| `Tasit` (Vehicle) | 0.934 | 0.902 | 0.941 | 0.768 |
| `Insan` (Human) | 0.902 | 0.870 | 0.907 | 0.714 |

---

## 6. Inference Benchmarks

| Hardware Target | Runtime Engine | Batch Size | Precision | Latency | Throughput |
|:---|:---|:---:|:---:|:---:|:---:|
| **NVIDIA Jetson Orin NX (16GB)** | TensorRT 8.6 | 1 | FP16 | **4.8 ms** | $\sim 208\text{ FPS}$ |
| **Intel Core i7-13700H (WSL2)** | ONNX Runtime | 1 | FP32 | **14.2 ms** | $\sim 70\text{ FPS}$ |
| **Raspberry Pi 5 (8GB)** | PyTorch CPU | 1 | FP32 | **48.6 ms** | $\sim 20\text{ FPS}$ |

*In the baseline Nadir runtime, YOLOv8 inference runs asynchronously at 10 Hz in a decoupled perception thread, maintaining zero impact on the 30 Hz ArUco PnP pose estimation loop.*
