#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import DataLoader, Subset
from torchvision import datasets, transforms

from azas_perception.cup_orientation_classifier import CupOrientationCNN


def stratified_split(dataset: datasets.ImageFolder, val_ratio: float, seed: int):
    by_class: dict[int, list[int]] = {}
    for index, (_, target) in enumerate(dataset.samples):
        by_class.setdefault(target, []).append(index)
    rng = random.Random(seed)
    train_indices: list[int] = []
    val_indices: list[int] = []
    for indices in by_class.values():
        rng.shuffle(indices)
        val_count = max(1, int(round(len(indices) * val_ratio))) if len(indices) > 1 else 0
        val_indices.extend(indices[:val_count])
        train_indices.extend(indices[val_count:])
    rng.shuffle(train_indices)
    rng.shuffle(val_indices)
    return train_indices, val_indices


def class_weights(dataset: datasets.ImageFolder, indices: list[int], device: str):
    counts = [0 for _ in dataset.classes]
    for index in indices:
        _, target = dataset.samples[index]
        counts[target] += 1
    total = sum(counts)
    weights = [total / max(count, 1) for count in counts]
    return torch.tensor(weights, dtype=torch.float32, device=device)


def run_epoch(model, loader, criterion, optimizer, device: str, train: bool):
    model.train(train)
    total_loss = 0.0
    total_correct = 0
    total_count = 0
    for images, targets in loader:
        images = images.to(device)
        targets = targets.to(device)
        if train:
            optimizer.zero_grad(set_to_none=True)
        logits = model(images)
        loss = criterion(logits, targets)
        if train:
            loss.backward()
            optimizer.step()
        total_loss += float(loss.item()) * int(targets.numel())
        total_correct += int((torch.argmax(logits, dim=1) == targets).sum().item())
        total_count += int(targets.numel())
    if total_count == 0:
        return 0.0, 0.0
    return total_loss / total_count, total_correct / total_count


def main() -> int:
    parser = argparse.ArgumentParser(description="Train upright/lying cup crop classifier.")
    parser.add_argument("--dataset-dir", default="/tmp/azas_cup_orientation_dataset")
    parser.add_argument("--output", default="/tmp/azas_cup_orientation_classifier.pt")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--image-size", type=int, default=128)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--patience", type=int, default=0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()

    random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = args.device

    train_transform = transforms.Compose([
        transforms.Resize((args.image_size, args.image_size)),
        transforms.ColorJitter(brightness=0.12, contrast=0.12, saturation=0.08),
        transforms.RandomRotation(degrees=6),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])
    eval_transform = transforms.Compose([
        transforms.Resize((args.image_size, args.image_size)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    dataset_dir = Path(args.dataset_dir)
    train_dataset_base = datasets.ImageFolder(str(dataset_dir), transform=train_transform)
    eval_dataset_base = datasets.ImageFolder(str(dataset_dir), transform=eval_transform)
    train_indices, val_indices = stratified_split(train_dataset_base, args.val_ratio, args.seed)
    if not train_indices or not val_indices:
        raise RuntimeError("dataset split is empty; collect more upright/lying images")

    train_loader = DataLoader(
        Subset(train_dataset_base, train_indices),
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=0,
    )
    val_loader = DataLoader(
        Subset(eval_dataset_base, val_indices),
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=0,
    )

    model = CupOrientationCNN(num_classes=len(train_dataset_base.classes)).to(device)
    criterion = nn.CrossEntropyLoss(weight=class_weights(train_dataset_base, train_indices, device))
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=1e-4)

    best_val_acc = -1.0
    best_state = None
    stale_epochs = 0
    print(f"classes={train_dataset_base.classes}")
    print(f"train={len(train_indices)} val={len(val_indices)} device={device}")
    for epoch in range(1, args.epochs + 1):
        train_loss, train_acc = run_epoch(model, train_loader, criterion, optimizer, device, train=True)
        val_loss, val_acc = run_epoch(model, val_loader, criterion, optimizer, device, train=False)
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
            stale_epochs = 0
        else:
            stale_epochs += 1
        print(
            f"epoch={epoch:03d} "
            f"train_loss={train_loss:.4f} train_acc={train_acc:.3f} "
            f"val_loss={val_loss:.4f} val_acc={val_acc:.3f}"
        )
        if args.patience > 0 and stale_epochs >= args.patience:
            print(f"early_stop epoch={epoch:03d} patience={args.patience} best_val_acc={best_val_acc:.3f}")
            break

    if best_state is None:
        raise RuntimeError("training did not produce a checkpoint")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": best_state,
            "class_names": train_dataset_base.classes,
            "image_size": args.image_size,
            "best_val_acc": best_val_acc,
        },
        str(output),
    )
    print(f"saved={output} best_val_acc={best_val_acc:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
