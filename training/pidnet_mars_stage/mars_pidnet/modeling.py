from __future__ import annotations

import sys
from pathlib import Path
import torch


def _upstream_path():
    path = Path(__file__).resolve().parents[1] / "upstream" / "PIDNet"
    if not path.is_dir():
        raise FileNotFoundError(f"official PIDNet checkout missing: {path}")
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def build_pidnet_s(num_classes: int, training: bool = True):
    _upstream_path()
    from models.pidnet import PIDNet
    return PIDNet(m=2, n=3, num_classes=num_classes, planes=32, ppm_planes=96,
                  head_planes=128, augment=training)


def load_matching_weights(model, checkpoint):
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
    state = payload.get("state_dict", payload)
    own = model.state_dict()
    loaded, skipped = {}, []
    for raw_key, value in state.items():
        candidates = [raw_key, raw_key.removeprefix("module."), raw_key[6:] if len(raw_key) > 6 else raw_key]
        key = next((k for k in candidates if k in own and own[k].shape == value.shape), None)
        if key is None:
            skipped.append(raw_key)
        else:
            loaded[key] = value
    result = model.load_state_dict(loaded, strict=False)
    return {"loaded_count": len(loaded), "skipped": skipped,
            "missing": list(result.missing_keys), "unexpected": list(result.unexpected_keys)}


def freeze_options(model, freeze_backbone=False, freeze_batch_norm_stats=False):
    if freeze_backbone:
        heads = ("final_layer", "seghead_p", "seghead_d")
        for name, param in model.named_parameters():
            param.requires_grad = name.startswith(heads)
    if freeze_batch_norm_stats:
        for module in model.modules():
            if isinstance(module, torch.nn.modules.batchnorm._BatchNorm):
                module.eval()
