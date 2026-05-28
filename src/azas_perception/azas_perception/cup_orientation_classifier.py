from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np

try:
    import torch
    from torch import nn
    from torchvision import models
except ImportError:  # pragma: no cover - depends on deployment environment
    torch = None
    nn = None
    models = None


DEFAULT_CLASS_NAMES = ["lying", "upright"]


if nn is not None:

    class CupOrientationCNN(nn.Module):
        def __init__(self, num_classes: int = 2):
            super().__init__()
            self.features = nn.Sequential(
                nn.Conv2d(3, 16, kernel_size=3, padding=1),
                nn.BatchNorm2d(16),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
                nn.Conv2d(16, 32, kernel_size=3, padding=1),
                nn.BatchNorm2d(32),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
                nn.Conv2d(32, 64, kernel_size=3, padding=1),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
                nn.Conv2d(64, 128, kernel_size=3, padding=1),
                nn.BatchNorm2d(128),
                nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d((1, 1)),
            )
            self.classifier = nn.Sequential(
                nn.Flatten(),
                nn.Dropout(p=0.25),
                nn.Linear(128, num_classes),
            )

        def forward(self, x):
            return self.classifier(self.features(x))

else:
    CupOrientationCNN = None


def crop_detection_bgr(
    image_bgr: np.ndarray,
    center_u: int,
    center_v: int,
    width: int,
    height: int,
    pad: float = 0.25,
) -> np.ndarray:
    image_height, image_width = image_bgr.shape[:2]
    crop_width = max(int(round(width * (1.0 + pad))), 1)
    crop_height = max(int(round(height * (1.0 + pad))), 1)
    x1 = max(center_u - crop_width // 2, 0)
    y1 = max(center_v - crop_height // 2, 0)
    x2 = min(x1 + crop_width, image_width)
    y2 = min(y1 + crop_height, image_height)
    x1 = max(x2 - crop_width, 0)
    y1 = max(y2 - crop_height, 0)
    return image_bgr[y1:y2, x1:x2].copy()


def preprocess_crop_bgr(crop_bgr: np.ndarray, image_size: int, device: str = "cpu"):
    if torch is None:
        raise RuntimeError("torch is not installed")
    if crop_bgr.size == 0:
        raise ValueError("empty crop")
    resized = cv2.resize(crop_bgr, (image_size, image_size), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    tensor = torch.from_numpy(rgb).to(torch.float32).permute(2, 0, 1) / 255.0
    mean = torch.tensor([0.485, 0.456, 0.406], dtype=torch.float32).view(3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], dtype=torch.float32).view(3, 1, 1)
    tensor = (tensor - mean) / std
    return tensor.unsqueeze(0).to(device)


def load_classifier_checkpoint(path: str | Path, device: str = "cpu", arch: str = "cnn"):
    if torch is None or CupOrientationCNN is None:
        raise RuntimeError("torch is not installed")
    normalized_arch = arch.strip().lower()
    if normalized_arch == "resnet18":
        return load_resnet18_classifier(path, device=device)
    if normalized_arch not in {"cnn", "small_cnn"}:
        raise ValueError(f"unsupported orientation classifier arch: {arch!r}")
    checkpoint = torch.load(str(path), map_location=device)
    class_names = list(checkpoint.get("class_names", DEFAULT_CLASS_NAMES))
    image_size = int(checkpoint.get("image_size", 128))
    model = CupOrientationCNN(num_classes=len(class_names)).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    return model, class_names, image_size


def load_resnet18_classifier(path: str | Path, device: str = "cpu"):
    if torch is None or nn is None or models is None:
        raise RuntimeError("torch/torchvision is not installed")
    class_names = DEFAULT_CLASS_NAMES
    image_size = 224
    state = torch.load(str(path), map_location=device)
    if isinstance(state, dict) and "model_state_dict" in state:
        class_names = list(state.get("class_names", DEFAULT_CLASS_NAMES))
        image_size = int(state.get("image_size", 224))
        state_dict = state["model_state_dict"]
    else:
        state_dict = state
    model = models.resnet18(weights=None)
    model.fc = nn.Linear(model.fc.in_features, len(class_names))
    model.load_state_dict(state_dict)
    model.to(device)
    model.eval()
    return model, class_names, image_size


def predict_crop_orientation(
    model,
    class_names: list[str],
    crop_bgr: np.ndarray,
    image_size: int,
    device: str = "cpu",
) -> tuple[str, float]:
    if torch is None:
        raise RuntimeError("torch is not installed")
    with torch.no_grad():
        logits = model(preprocess_crop_bgr(crop_bgr, image_size=image_size, device=device))
        probs = torch.softmax(logits, dim=1)[0]
        confidence, index = torch.max(probs, dim=0)
    return class_names[int(index.item())], float(confidence.item())


def classifier_available(path: str | Path) -> bool:
    return bool(str(path).strip()) and Path(path).exists()
