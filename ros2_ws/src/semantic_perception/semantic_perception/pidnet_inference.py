"""Checkpoint loading and image inference for the hazard5 PIDNet-S model."""

from pathlib import Path
import time
from typing import Any, Dict, Tuple

import numpy as np
import torch
import torch.nn.functional as functional

from semantic_perception.pidnet_contract import NUM_CLASSES, validate_prediction
from semantic_perception.pidnet_model import build_pidnet_s


SOURCE_SIZE = (640, 360)
MODEL_SIZE = (640, 384)
PAD_TOP = 12
PAD_BOTTOM = 12
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class PIDNetInference:
    """Load one V3 checkpoint and run deterministic RGB inference."""

    def __init__(
        self,
        model_path: str,
        device: str = 'cuda',
        precision: str = 'fp16',
        allow_fp32_fallback: bool = True,
    ) -> None:
        checkpoint = Path(model_path).expanduser().resolve()
        if not checkpoint.is_file():
            raise FileNotFoundError(f'PIDNet checkpoint not found: {checkpoint}')
        if precision not in ('fp16', 'fp32'):
            raise ValueError('precision must be fp16 or fp32')
        if not device:
            raise ValueError('device must be a non-empty torch device string')

        self.device = torch.device(device)
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA was requested but is unavailable')
        if precision == 'fp16' and self.device.type != 'cuda':
            raise ValueError('fp16 inference is supported only on CUDA')

        payload = self._load_payload(checkpoint)
        self._validate_checkpoint(payload)
        self.model = build_pidnet_s(NUM_CLASSES)
        load_report = self._load_state_dict(payload['state_dict'])
        self.model.eval().to(self.device)

        self.checkpoint = checkpoint
        self.allow_fp32_fallback = bool(allow_fp32_fallback)
        self.precision = 'fp32'
        self.last_fallback_reason = None
        self._set_precision(precision)
        if self.device.type == 'cuda':
            torch.backends.cudnn.benchmark = True

        self.metadata: Dict[str, Any] = {
            'checkpoint': str(checkpoint),
            'epoch': payload.get('epoch'),
            'val_miou': payload.get('metrics', {}).get('miou'),
            'num_classes': payload.get('num_classes'),
            'ignore_index': payload.get('ignore_index'),
            'loaded_tensors': load_report['loaded_tensors'],
            'skipped_auxiliary_tensors': load_report['skipped_auxiliary_tensors'],
            'device': str(self.device),
            'precision': self.precision,
        }

    @staticmethod
    def _load_payload(checkpoint: Path) -> Dict[str, Any]:
        try:
            payload = torch.load(
                checkpoint, map_location='cpu', weights_only=False)
        except TypeError:
            payload = torch.load(checkpoint, map_location='cpu')
        if not isinstance(payload, dict):
            raise RuntimeError('checkpoint payload must be a dictionary')
        return payload

    @staticmethod
    def _validate_checkpoint(payload: Dict[str, Any]) -> None:
        if payload.get('num_classes') != NUM_CLASSES:
            raise RuntimeError(
                f'checkpoint num_classes must be {NUM_CLASSES}, '
                f"got {payload.get('num_classes')}")
        if payload.get('ignore_index') != 255:
            raise RuntimeError('checkpoint ignore_index must be 255')
        if not isinstance(payload.get('state_dict'), dict):
            raise RuntimeError('checkpoint does not contain a state_dict')
        config = payload.get('config')
        if not isinstance(config, dict):
            raise RuntimeError('checkpoint does not contain a config dictionary')
        if tuple(config.get('input_size', ())) != MODEL_SIZE:
            raise RuntimeError(
                f"checkpoint input_size mismatch: {config.get('input_size')}")
        if tuple(config.get('source_size', ())) != SOURCE_SIZE:
            raise RuntimeError(
                f"checkpoint source_size mismatch: {config.get('source_size')}")

    def _load_state_dict(self, state_dict: Dict[str, torch.Tensor]) -> Dict[str, int]:
        expected = self.model.state_dict()
        compatible = {}
        skipped = []
        for raw_key, value in state_dict.items():
            candidates = (
                raw_key,
                raw_key.removeprefix('module.'),
                raw_key[6:] if len(raw_key) > 6 else raw_key,
            )
            key = next(
                (
                    candidate for candidate in candidates
                    if candidate in expected
                    and expected[candidate].shape == value.shape
                ),
                None,
            )
            if key is None:
                skipped.append(raw_key)
            else:
                compatible[key] = value

        result = self.model.load_state_dict(compatible, strict=False)
        if result.missing_keys or result.unexpected_keys:
            raise RuntimeError(
                'checkpoint/model mismatch: '
                f'missing={result.missing_keys} unexpected={result.unexpected_keys}')
        invalid_skips = [
            key for key in skipped
            if not key.removeprefix('module.').startswith(
                ('seghead_p.', 'seghead_d.'))
        ]
        if invalid_skips:
            raise RuntimeError(
                f'checkpoint contains unmatched non-auxiliary tensors: {invalid_skips}')
        return {
            'loaded_tensors': len(compatible),
            'skipped_auxiliary_tensors': len(skipped),
        }

    def _set_precision(self, precision: str) -> None:
        if precision == 'fp16':
            self.model.half()
        else:
            self.model.float()
        self.precision = precision
        if hasattr(self, 'metadata'):
            self.metadata['precision'] = precision

    @staticmethod
    def _preprocess(rgb: np.ndarray) -> np.ndarray:
        if rgb.dtype != np.uint8 or rgb.shape != (360, 640, 3):
            raise ValueError(
                'PIDNet input must be a 640x360 uint8 RGB image, '
                f'got shape={rgb.shape} dtype={rgb.dtype}')
        padded = np.pad(
            rgb,
            ((PAD_TOP, PAD_BOTTOM), (0, 0), (0, 0)),
            mode='constant',
            constant_values=0,
        )
        normalized = (padded.astype(np.float32) / 255.0 - MEAN) / STD
        return np.ascontiguousarray(normalized.transpose(2, 0, 1))

    def _predict_once(self, rgb: np.ndarray) -> Tuple[np.ndarray, float]:
        array = self._preprocess(rgb)
        tensor = torch.from_numpy(array).unsqueeze(0).to(
            self.device,
            dtype=torch.float16 if self.precision == 'fp16' else torch.float32,
            non_blocking=True,
        )
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)
        started = time.perf_counter()
        with torch.inference_mode():
            logits = self.model(tensor)
            if logits.ndim != 4 or logits.shape[1] != NUM_CLASSES:
                raise RuntimeError(
                    f'unexpected PIDNet logits shape: {tuple(logits.shape)}')
            logits = functional.interpolate(
                logits,
                size=(MODEL_SIZE[1], MODEL_SIZE[0]),
                mode='bilinear',
                align_corners=False,
            )
            prediction = logits.argmax(dim=1)[0, PAD_TOP:-PAD_BOTTOM]
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)
        inference_ms = (time.perf_counter() - started) * 1000.0
        mask = prediction.to(device='cpu', dtype=torch.uint8).numpy()
        return validate_prediction(mask), inference_ms

    def predict(self, rgb: np.ndarray) -> Tuple[np.ndarray, float]:
        """Return a 640x360 uint8 prediction and synchronized inference time."""
        try:
            return self._predict_once(rgb)
        except RuntimeError as error:
            if self.precision != 'fp16' or not self.allow_fp32_fallback:
                raise
            self.last_fallback_reason = f'{type(error).__name__}: {error}'
            self._set_precision('fp32')
            if self.device.type == 'cuda':
                torch.cuda.empty_cache()
            return self._predict_once(rgb)
