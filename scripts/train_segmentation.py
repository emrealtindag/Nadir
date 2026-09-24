"""
Deep Learning Segmentation Training Pipeline (PyTorch) for Scandium.

This script demonstrates advanced fine-tuning of a UNet/DeepLabV3 architecture
for landing zone segmentation, utilizing PyTorch and torchvision.

It's designed to train on aerial datasets to classify pixels as:
- Safe Landing Zone (Class 1)
- Unsafe/Obstacle (Class 0)
"""

import os
import argparse
import logging
from typing import Tuple

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import Dataset, DataLoader
    import torchvision.transforms.functional as TF
    from torchvision.models.segmentation import deeplabv3_resnet50, DeepLabV3_ResNet50_Weights
    import numpy as np
    import cv2
except ImportError:
    print("PyTorch or related libraries not found. Run: pip install torch torchvision opencv-python")
    exit(1)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("LandabilityTrainer")

class LandingZoneDataset(Dataset):
    """
    Custom Dataset for reading UAV landing images and corresponding semantic masks.
    """
    def __init__(self, images_dir: str, masks_dir: str, img_size: Tuple[int, int] = (256, 256)):
        self.images_dir = images_dir
        self.masks_dir = masks_dir
        self.img_size = img_size
        self.images = [f for f in os.listdir(images_dir) if f.endswith('.jpg') or f.endswith('.png')]

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        img_name = self.images[idx]
        img_path = os.path.join(self.images_dir, img_name)
        mask_path = os.path.join(self.masks_dir, img_name.replace('.jpg', '.png'))

        image = cv2.imread(img_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

        image = cv2.resize(image, self.img_size)
        mask = cv2.resize(mask, self.img_size, interpolation=cv2.INTER_NEAREST)

        # Normalize image and convert to tensor (C, H, W)
        image = TF.to_tensor(image)
        
        # Mask tensor (1 for safe, 0 for obstacle)
        mask = torch.from_numpy(mask).long()
        mask = torch.where(mask > 128, torch.tensor(1), torch.tensor(0))

        return image, mask

def build_model(num_classes: int = 2) -> nn.Module:
    """Builds a DeepLabV3 model with a custom classification head."""
    logger.info("Initializing DeepLabV3 ResNet50 Architecture...")
    model = deeplabv3_resnet50(weights=DeepLabV3_ResNet50_Weights.DEFAULT)
    # Replace the classifier for our binary task
    model.classifier[4] = nn.Conv2d(256, num_classes, kernel_size=(1, 1), stride=(1, 1))
    return model

def train_epoch(model: nn.Module, loader: DataLoader, criterion: nn.Module, optimizer: optim.Optimizer, device: torch.device):
    model.train()
    total_loss = 0.0
    for images, masks in loader:
        images = images.to(device)
        masks = masks.to(device)

        optimizer.zero_grad()
        outputs = model(images)['out']
        loss = criterion(outputs, masks)
        
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
        
    return total_loss / len(loader)

def main():
    parser = argparse.ArgumentParser(description="Train Landability Segmentation Model")
    parser.add_argument("--epochs", type=int, default=50, help="Number of training epochs")
    parser.add_argument("--batch-size", type=int, default=16, help="Batch size")
    parser.add_argument("--lr", type=float, default=1e-4, help="Learning rate")
    parser.add_argument("--data-dir", type=str, default="./data/landing_zones", help="Path to dataset")
    parser.add_argument("--output", type=str, default="landability_model.onnx", help="Output ONNX path")
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"Using device: {device}")

    # Dataset mock check (in real scenario, dataset should exist)
    img_dir = os.path.join(args.data_dir, "images")
    mask_dir = os.path.join(args.data_dir, "masks")
    
    if not os.path.exists(img_dir) or not os.path.exists(mask_dir):
        logger.warning(f"Dataset not found at {args.data_dir}. Running in dummy mode to demonstrate compilation.")
        # Dummy data for demonstration
        inputs = torch.randn(args.batch_size, 3, 256, 256).to(device)
        targets = torch.randint(0, 2, (args.batch_size, 256, 256)).to(device)
        loader = [(inputs, targets)]
    else:
        dataset = LandingZoneDataset(img_dir, mask_dir)
        loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True, num_workers=4)

    model = build_model(num_classes=2).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=args.lr)

    logger.info("Starting training loop...")
    for epoch in range(1, args.epochs + 1):
        loss = train_epoch(model, loader, criterion, optimizer, device)
        if epoch % 5 == 0:
            logger.info(f"Epoch [{epoch}/{args.epochs}] - Loss: {loss:.4f}")

    logger.info("Training complete. Exporting to ONNX...")
    model.eval()
    dummy_input = torch.randn(1, 3, 256, 256).to(device)
    
    torch.onnx.export(
        model, 
        dummy_input, 
        args.output, 
        export_params=True, 
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'], 
        output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
    )
    logger.info(f"Model exported successfully to {args.output}")

if __name__ == "__main__":
    main()
