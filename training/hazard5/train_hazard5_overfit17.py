#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import logging
import random
import sys
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from mars_pidnet.modeling import build_pidnet_s

CLASS_NAMES = ["other", "hill_candidate", "crater_candidate", "step_candidate", "rover"]
COLORS = np.array([
    [128, 128, 128], [40, 110, 255], [255, 50, 50],
    [255, 220, 20], [30, 220, 80],
], dtype=np.uint8)
IGNORE_COLOR = np.array([180, 40, 210], dtype=np.uint8)
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True


class Hazard17Dataset(Dataset):
    def __init__(self, audit_report: Path):
        report = json.loads(audit_report.read_text(encoding="utf-8"))
        if not report.get("all_17_participate"):
            raise ValueError("pair audit did not pass all_17_participate")
        self.pairs = report["pairs"]
        if len(self.pairs) != 17 or any(x["status"] != "VALID" for x in self.pairs):
            raise ValueError("training requires exactly 17 VALID pairs")

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, index: int) -> dict:
        pair = self.pairs[index]
        rgb = np.asarray(Image.open(pair["rgb_path"]).convert("RGB"), dtype=np.uint8)
        mask = np.asarray(Image.open(pair["mask_path"]), dtype=np.uint8)
        rgb = np.pad(rgb, ((12, 12), (0, 0), (0, 0)), constant_values=0)
        mask = np.pad(mask, ((12, 12), (0, 0)), constant_values=255)
        normalized = (rgb.astype(np.float32) / 255.0 - MEAN) / STD
        return {
            "image": torch.from_numpy(normalized.transpose(2, 0, 1)).float(),
            "mask": torch.from_numpy(mask.astype(np.int64)),
            "id": pair["sample_id"], "rgb_path": pair["rgb_path"],
        }


def set_bn_eval(model: torch.nn.Module) -> None:
    for module in model.modules():
        if isinstance(module, torch.nn.modules.batchnorm._BatchNorm):
            module.eval()


def main_logits(outputs):
    if torch.is_tensor(outputs):
        return outputs
    if not isinstance(outputs, (list, tuple)) or len(outputs) < 2:
        raise RuntimeError(f"unexpected PIDNet output type: {type(outputs)}")
    return outputs[1]


def load_cityscapes(model: torch.nn.Module, checkpoint: Path) -> dict:
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
    state = payload.get("state_dict", payload)
    own = model.state_dict()
    loaded = {}
    skipped = []
    for raw_key, value in state.items():
        candidates = [raw_key, raw_key.removeprefix("module."), raw_key[6:] if len(raw_key) > 6 else raw_key]
        key = next((name for name in candidates if name in own and own[name].shape == value.shape), None)
        if key is None:
            skipped.append({"key": raw_key, "shape": list(value.shape)})
        else:
            loaded[key] = value
    result = model.load_state_dict(loaded, strict=False)
    heads = ("final_layer", "seghead_p", "seghead_d")
    backbone = {key: value for key, value in loaded.items() if not key.startswith(heads)}
    head = {key: value for key, value in loaded.items() if key.startswith(heads)}
    return {
        "checkpoint": str(checkpoint), "source_tensor_count": len(state),
        "loaded_tensor_count": len(loaded), "loaded_parameter_numel": sum(v.numel() for v in loaded.values()),
        "backbone_loaded_tensor_count": len(backbone),
        "backbone_loaded_parameter_numel": sum(v.numel() for v in backbone.values()),
        "head_loaded_tensor_count": len(head), "head_loaded_parameter_numel": sum(v.numel() for v in head.values()),
        "skipped_count": len(skipped), "skipped": skipped,
        "missing_after_load": list(result.missing_keys), "unexpected_after_load": list(result.unexpected_keys),
    }


def boundary_target(mask: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    valid = mask != 255
    edge = torch.zeros_like(valid)
    horizontal = valid[:, :, 1:] & valid[:, :, :-1] & (mask[:, :, 1:] != mask[:, :, :-1])
    vertical = valid[:, 1:, :] & valid[:, :-1, :] & (mask[:, 1:, :] != mask[:, :-1, :])
    edge[:, :, 1:] |= horizontal
    edge[:, :, :-1] |= horizontal
    edge[:, 1:, :] |= vertical
    edge[:, :-1, :] |= vertical
    return edge.float().unsqueeze(1), valid.unsqueeze(1)


def confusion(pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    valid = target != 255
    encoded = target[valid] * 5 + pred[valid]
    return torch.bincount(encoded, minlength=25).reshape(5, 5).cpu()


def metrics_from_matrix(matrix: torch.Tensor, pred_counts: torch.Tensor, ignored: int) -> dict:
    matrix_f = matrix.double()
    tp = matrix_f.diag()
    gt = matrix_f.sum(1)
    pred = matrix_f.sum(0)
    union = gt + pred - tp
    iou = torch.where(union > 0, tp / union, torch.nan)
    precision = torch.where(pred > 0, tp / pred, torch.nan)
    recall = torch.where(gt > 0, tp / gt, torch.nan)
    miou = torch.nanmean(iou).item()
    total_pred = int(pred_counts.sum())
    fractions = (pred_counts.double() / max(total_pred, 1)).tolist()
    dominant_id = int(torch.argmax(pred_counts)) if total_pred else -1
    return {
        "confusion_matrix": matrix.tolist(),
        "per_class_iou": [None if torch.isnan(x) else x.item() for x in iou],
        "per_class_precision": [None if torch.isnan(x) else x.item() for x in precision],
        "per_class_recall": [None if torch.isnan(x) else x.item() for x in recall],
        "miou": miou, "prediction_pixel_counts": pred_counts.tolist(),
        "prediction_fractions": fractions, "dominant_class": CLASS_NAMES[dominant_id] if dominant_id >= 0 else None,
        "class_collapse": bool(total_pred and pred_counts[dominant_id] / total_pred > 0.95),
        "all_other": bool(total_pred and pred_counts[0] == total_pred),
        "all_hill": bool(total_pred and pred_counts[1] == total_pred),
        "valid_metric_pixels": int(matrix.sum()), "ignored_pixels_excluded": ignored,
    }


def colorize(mask: np.ndarray) -> np.ndarray:
    output = np.zeros((*mask.shape, 3), dtype=np.uint8)
    valid = mask != 255
    output[valid] = COLORS[mask[valid]]
    output[~valid] = IGNORE_COLOR
    return output


def save_final_visuals(output: Path, sample_id: str, rgb_path: str,
                       gt_padded: np.ndarray, pred_padded: np.ndarray) -> None:
    rgb = np.asarray(Image.open(rgb_path).convert("RGB"), dtype=np.uint8)
    gt = gt_padded[12:372]
    pred = pred_padded[12:372]
    gt_color, pred_color = colorize(gt), colorize(pred)
    gt_overlay = (rgb.astype(np.float32) * 0.6 + gt_color.astype(np.float32) * 0.4).astype(np.uint8)
    pred_overlay = (rgb.astype(np.float32) * 0.6 + pred_color.astype(np.float32) * 0.4).astype(np.uint8)
    destinations = {
        "rgb": Image.fromarray(rgb, mode="RGB"), "gt": Image.fromarray(gt, mode="L"),
        "prediction": Image.fromarray(pred, mode="L"),
        "gt_overlay": Image.fromarray(gt_overlay, mode="RGB"),
        "prediction_overlay": Image.fromarray(pred_overlay, mode="RGB"),
    }
    for folder, image in destinations.items():
        directory = output / "final_visuals" / folder
        directory.mkdir(parents=True, exist_ok=True)
        image.save(directory / f"{sample_id}.png")


@torch.no_grad()
def evaluate(model: torch.nn.Module, loader: DataLoader, device: torch.device,
             output: Path, step: int, save_periodic: bool, save_final: bool = False) -> dict:
    model.eval()
    matrix = torch.zeros(5, 5, dtype=torch.int64)
    pred_counts = torch.zeros(5, dtype=torch.int64)
    ignored = 0
    periodic_dir = output / "periodic_predictions" / f"iter_{step:06d}"
    if save_periodic:
        periodic_dir.mkdir(parents=True, exist_ok=True)
    for batch in loader:
        images = batch["image"].to(device)
        targets = batch["mask"]
        logits = main_logits(model(images))
        logits = F.interpolate(logits, size=targets.shape[-2:], mode="bilinear", align_corners=False)
        predictions = logits.argmax(1).cpu()
        for index in range(targets.shape[0]):
            target, prediction = targets[index], predictions[index]
            matrix += confusion(prediction, target)
            valid = target != 255
            pred_counts += torch.bincount(prediction[valid], minlength=5)
            ignored += int((~valid).sum())
            sid = batch["id"][index]
            if save_periodic:
                Image.fromarray(prediction.numpy()[12:372].astype(np.uint8), mode="L").save(periodic_dir / f"{sid}.png")
            if save_final:
                save_final_visuals(output, sid, batch["rgb_path"][index],
                                   target.numpy().astype(np.uint8), prediction.numpy().astype(np.uint8))
    return metrics_from_matrix(matrix, pred_counts, ignored)


def save_checkpoint(path: Path, model: torch.nn.Module, optimizer: torch.optim.Optimizer,
                    step: int, metrics: dict, losses: list[dict], weight_load: dict) -> None:
    torch.save({"state_dict": model.state_dict(), "optimizer": optimizer.state_dict(),
                "iteration": step, "num_classes": 5, "ignore_index": 255,
                "metrics": metrics, "losses": losses, "weight_load": weight_load}, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit-report", type=Path, required=True)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--initial-iterations", type=int, default=1000)
    parser.add_argument("--max-iterations", type=int, default=3000)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=304)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--grad-clip", type=float, default=0.0)
    parser.add_argument("--step-weight-multiplier", type=float, default=1.0)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        handlers=[logging.FileHandler(output / "training.log", encoding="utf-8"), logging.StreamHandler(sys.stdout)])
    log = logging.getLogger("hazard5_overfit17")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA GPU is required")
    if args.batch_size != 2:
        raise ValueError("hazard5 overfit17 requires batch_size=2")
    seed_everything(args.seed)
    device = torch.device("cuda:0")
    dataset = Hazard17Dataset(args.audit_report)
    generator = torch.Generator().manual_seed(args.seed)
    train_loader = DataLoader(dataset, batch_size=2, shuffle=True, num_workers=0, generator=generator, drop_last=False)
    eval_loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=0)

    model = build_pidnet_s(5, training=True).to(device)
    weight_load = load_cityscapes(model, args.weights)
    resume_payload = None
    start_iteration = 0
    if args.resume:
        resume_payload = torch.load(args.resume, map_location=device, weights_only=False)
        if resume_payload.get("num_classes") != 5 or resume_payload.get("ignore_index") != 255:
            raise ValueError("resume checkpoint is not hazard5/ignore255")
        model.load_state_dict(resume_payload["state_dict"])
        start_iteration = int(resume_payload["iteration"])
        if start_iteration >= args.max_iterations:
            raise ValueError("resume iteration must be lower than max_iterations")
    model.eval()
    probe = torch.zeros(2, 3, 384, 640, device=device)
    with torch.no_grad():
        probe_outputs = model(probe)
    output_shapes = [list(x.shape) for x in probe_outputs] if isinstance(probe_outputs, (list, tuple)) else [list(probe_outputs.shape)]
    if any(shape[1] != 5 for shape in output_shapes[:2]):
        raise RuntimeError(f"classification output channels are not 5: {output_shapes}")
    del probe, probe_outputs
    (output / "weight_load_report.json").write_text(json.dumps(weight_load, indent=2), encoding="utf-8")

    pixel_counts = torch.zeros(5, dtype=torch.float64)
    for pair in dataset.pairs:
        mask = np.asarray(Image.open(pair["mask_path"]), dtype=np.uint8)
        for value in range(5):
            pixel_counts[value] += int(np.sum(mask == value))
    raw_weights = torch.sqrt(pixel_counts.sum() / (5.0 * pixel_counts.clamp_min(1)))
    class_weights = (raw_weights / raw_weights.mean()).clamp(0.25, 3.0).float().to(device)
    class_weights[3] *= args.step_weight_multiplier
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    run_config = {
        "task": "PIDNet-S hazard5 overfit17", "num_classes": 5, "class_names": CLASS_NAMES,
        "ignore_index": 255, "input_size": [640, 384], "source_size": [640, 360],
        "padding": {"top": 12, "bottom": 12, "mask_value": 255},
        "augmentation": False, "batch_size": 2, "seed": args.seed, "lr": args.lr,
        "initial_iterations": args.initial_iterations, "max_iterations": args.max_iterations,
        "resume": str(args.resume) if args.resume else None, "start_iteration": start_iteration,
        "grad_clip": args.grad_clip, "step_weight_multiplier": args.step_weight_multiplier,
        "train_samples": len(dataset), "eval_samples": len(dataset),
        "class_weights": class_weights.cpu().tolist(), "output_shapes": output_shapes,
        "gpu": torch.cuda.get_device_name(0), "torch": torch.__version__, "cuda_build": torch.version.cuda,
    }
    (output / "run_config.json").write_text(json.dumps(run_config, indent=2), encoding="utf-8")
    log.info("run_config=%s", json.dumps(run_config))
    log.info("weight_load=%s", json.dumps(weight_load))

    initial_metrics = evaluate(model, eval_loader, device, output, start_iteration, save_periodic=True)
    (output / "metrics_initial.json").write_text(json.dumps(initial_metrics, indent=2), encoding="utf-8")
    losses: list[dict] = list(resume_payload.get("losses", [])) if resume_payload else []
    seen = Counter()
    best_miou = initial_metrics["miou"]
    best_step = start_iteration
    iterator = iter(train_loader)
    actual_iterations = args.max_iterations
    save_checkpoint(output / "best_checkpoint.pt", model, optimizer, start_iteration,
                    initial_metrics, losses, weight_load)

    for step in range(start_iteration + 1, args.max_iterations + 1):
        try:
            batch = next(iterator)
        except StopIteration:
            iterator = iter(train_loader)
            batch = next(iterator)
        seen.update(batch["id"])
        images = batch["image"].to(device)
        targets = batch["mask"].to(device)
        model.train()
        set_bn_eval(model)
        outputs = model(images)
        if len(outputs) < 3 or outputs[0].shape[1] != 5 or outputs[1].shape[1] != 5:
            raise RuntimeError(f"PIDNet heads invalid at step {step}: {[tuple(x.shape) for x in outputs]}")
        aux = F.interpolate(outputs[0], size=targets.shape[-2:], mode="bilinear", align_corners=False)
        main = F.interpolate(outputs[1], size=targets.shape[-2:], mode="bilinear", align_corners=False)
        boundary = F.interpolate(outputs[2], size=targets.shape[-2:], mode="bilinear", align_corners=False)
        aux_loss = F.cross_entropy(aux, targets, weight=class_weights, ignore_index=255)
        main_loss = F.cross_entropy(main, targets, weight=class_weights, ignore_index=255)
        boundary_gt, boundary_valid = boundary_target(targets)
        boundary_raw = F.binary_cross_entropy_with_logits(boundary, boundary_gt, reduction="none")
        boundary_loss = boundary_raw[boundary_valid].mean()
        loss = 0.4 * aux_loss + main_loss + 0.1 * boundary_loss
        if not torch.isfinite(loss):
            raise RuntimeError(f"non-finite loss at iteration {step}")
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        if args.grad_clip > 0:
            torch.nn.utils.clip_grad_norm_(model.parameters(), args.grad_clip)
        optimizer.step()
        entry = {"iteration": step, "loss": loss.item(), "main_loss": main_loss.item(),
                 "aux_loss": aux_loss.item(), "boundary_loss": boundary_loss.item()}
        losses.append(entry)
        if step == 1 or step % 20 == 0:
            log.info("iteration=%d loss=%.6f main=%.6f aux=%.6f boundary=%.6f",
                     step, entry["loss"], entry["main_loss"], entry["aux_loss"], entry["boundary_loss"])

        if step % 50 == 0 or step == args.initial_iterations or step == args.max_iterations:
            save_periodic = step % 250 == 0 or step in {args.initial_iterations, args.max_iterations}
            current = evaluate(model, eval_loader, device, output, step, save_periodic=save_periodic)
            log.info("evaluation iteration=%d miou=%.6f iou=%s collapse=%s fractions=%s", step, current["miou"],
                     current["per_class_iou"], current["class_collapse"], current["prediction_fractions"])
            if current["miou"] > best_miou:
                best_miou, best_step = current["miou"], step
                save_checkpoint(output / "best_checkpoint.pt", model, optimizer, step, current, losses, weight_load)
            if start_iteration < args.initial_iterations and step == args.initial_iterations:
                crater_iou = current["per_class_iou"][2] or 0.0
                step_iou = current["per_class_iou"][3] or 0.0
                obvious = current["miou"] >= 0.90 and crater_iou >= 0.80 and step_iou >= 0.80 and not current["class_collapse"]
                if obvious:
                    actual_iterations = step
                    log.info("overfit criterion met at %d; stopping before max_iterations", step)
                    break
                log.info("overfit criterion not met at %d; continuing to max_iterations=%d", step, args.max_iterations)

    final_metrics = evaluate(model, eval_loader, device, output, actual_iterations,
                             save_periodic=True, save_final=True)
    save_checkpoint(output / "last_checkpoint.pt", model, optimizer, actual_iterations,
                    final_metrics, losses, weight_load)
    final_metrics.update({
        "actual_iterations": actual_iterations, "best_miou": best_miou, "best_iteration": best_step,
        "initial_loss": losses[0]["loss"], "final_loss": losses[-1]["loss"],
        "all_17_seen_in_training": set(seen) == {pair["sample_id"] for pair in dataset.pairs},
        "sample_seen_counts": dict(seen), "train_eval_same_17": True,
        "generalization_warning": "train and eval are the same 17 samples; metrics measure memorization only",
        "ignore_index": 255, "classification_output_channels": 5,
    })
    (output / "metrics_final.json").write_text(json.dumps(final_metrics, indent=2), encoding="utf-8")
    with (output / "loss.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(losses[0]))
        writer.writeheader(); writer.writerows(losses)
    with (output / "per_class_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle); writer.writerow(["class_id", "class_name", "iou", "precision", "recall"])
        for value, name in enumerate(CLASS_NAMES):
            writer.writerow([value, name, final_metrics["per_class_iou"][value],
                             final_metrics["per_class_precision"][value], final_metrics["per_class_recall"][value]])
    with (output / "confusion_matrix.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle); writer.writerow(["gt\\pred", *CLASS_NAMES])
        for name, row in zip(CLASS_NAMES, final_metrics["confusion_matrix"]): writer.writerow([name, *row])

    plt.figure(figsize=(8, 4)); plt.plot([x["iteration"] for x in losses], [x["loss"] for x in losses])
    plt.xlabel("iteration"); plt.ylabel("loss"); plt.tight_layout(); plt.savefig(output / "loss_curve.png", dpi=160); plt.close()
    matrix = np.asarray(final_metrics["confusion_matrix"])
    plt.figure(figsize=(7, 6)); plt.imshow(matrix, cmap="Blues"); plt.colorbar();
    plt.xticks(range(5), CLASS_NAMES, rotation=30, ha="right"); plt.yticks(range(5), CLASS_NAMES)
    plt.xlabel("prediction"); plt.ylabel("ground truth"); plt.tight_layout();
    plt.savefig(output / "confusion_matrix.png", dpi=160); plt.close()
    log.info("final_metrics=%s", json.dumps(final_metrics))
    log.info("best_checkpoint=%s last_checkpoint=%s", output / "best_checkpoint.pt", output / "last_checkpoint.pt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
