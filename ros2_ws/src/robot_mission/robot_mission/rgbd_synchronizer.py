"""ROS-independent bounded RGB/depth timestamp matching and diagnostics."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import statistics
from typing import Any, Deque, Dict, Iterable, List, Optional


DIRECT_SLOP = 'direct_slop'
FIXED_OFFSET = 'fixed_offset'
SYNC_MODES = (DIRECT_SLOP, FIXED_OFFSET)


class SensorSyncState(str, Enum):
    """Recoverable health of the incoming image streams."""

    SYNC_OK = 'SYNC_OK'
    RGB_ONLY = 'RGB_ONLY'
    DEPTH_STALE = 'DEPTH_STALE'
    NO_RGB = 'NO_RGB'
    SENSOR_SILENCE = 'SENSOR_SILENCE'


class ConsecutiveMatchGate:
    """Require a small run of matches before declaring depth recovered."""

    def __init__(self, required_count: int = 2) -> None:
        if required_count < 1:
            raise ValueError('required_count must be positive')
        self.required_count = required_count
        self.count = 0

    def mark_match(self) -> None:
        self.count = min(self.required_count, self.count + 1)

    def mark_unhealthy(self) -> None:
        self.count = 0

    @property
    def ready(self) -> bool:
        return self.count >= self.required_count


@dataclass(frozen=True)
class BufferedMessage:
    message: Any
    stamp_sec: float
    logical_stamp_sec: float
    arrival_time: float


@dataclass(frozen=True)
class SyncMatch:
    rgb: BufferedMessage
    depth: BufferedMessage
    raw_delta_ms: float
    corrected_delta_ms: float


@dataclass(frozen=True)
class SyncSample:
    """One matched pair or one buffer eviction for CSV diagnostics."""

    sync_mode: str
    wall_time: float
    rgb_stamp_sec: Optional[float]
    depth_stamp_sec: Optional[float]
    raw_delta_ms: Optional[float]
    depth_stamp_offset_ms: float
    corrected_delta_ms: Optional[float]
    matched: bool
    rgb_buffer_size: int
    depth_buffer_size: int
    reason: str


def _percentile_95(values: Iterable[float]) -> Optional[float]:
    ordered = sorted(values)
    if not ordered:
        return None
    index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return float(ordered[index])


def _age(now: float, arrival: Optional[float]) -> float:
    return math.inf if arrival is None else max(0.0, now - arrival)


def classify_sensor_state(
    now: float,
    rgb_arrival_time: Optional[float],
    depth_arrival_time: Optional[float],
    matched_depth_arrival_time: Optional[float],
    *,
    depth_freshness_limit_sec: float,
    sensor_stop_timeout_sec: float,
    sensor_warn_timeout_sec: float,
) -> SensorSyncState:
    """Classify stream health without making any state terminal."""
    rgb_age = _age(now, rgb_arrival_time)
    depth_age = _age(now, depth_arrival_time)
    matched_depth_age = _age(now, matched_depth_arrival_time)
    if rgb_age > sensor_warn_timeout_sec and depth_age > sensor_warn_timeout_sec:
        return SensorSyncState.SENSOR_SILENCE
    if rgb_age > sensor_stop_timeout_sec:
        return SensorSyncState.NO_RGB
    if (depth_age > sensor_stop_timeout_sec
            or matched_depth_age > sensor_stop_timeout_sec):
        return SensorSyncState.DEPTH_STALE
    if matched_depth_age > depth_freshness_limit_sec:
        return SensorSyncState.RGB_ONLY
    return SensorSyncState.SYNC_OK


class RgbDepthSynchronizer:
    """Greedy nearest-neighbour matcher with bounded, expiring buffers.

    For ``fixed_offset`` only the internal logical Depth time is shifted:

    ``corrected_depth_stamp = original_depth_stamp + depth_stamp_offset_ms``.

    The ROS message and its header are never modified.
    """

    def __init__(
        self,
        mode: str,
        slop_ms: float,
        *,
        depth_stamp_offset_ms: float = 0.0,
        queue_size: int = 10,
        max_buffer_age_ms: float = 1000.0,
    ) -> None:
        if mode not in SYNC_MODES:
            raise ValueError(f'unsupported sync mode: {mode}')
        if not math.isfinite(slop_ms) or slop_ms <= 0.0:
            raise ValueError('slop_ms must be finite and positive')
        if not math.isfinite(depth_stamp_offset_ms):
            raise ValueError('depth_stamp_offset_ms must be finite')
        if queue_size < 1:
            raise ValueError('queue_size must be positive')
        if not math.isfinite(max_buffer_age_ms) or max_buffer_age_ms <= 0.0:
            raise ValueError('max_buffer_age_ms must be finite and positive')
        self.mode = mode
        self.slop_ms = float(slop_ms)
        self.depth_stamp_offset_ms = (
            float(depth_stamp_offset_ms) if mode == FIXED_OFFSET else 0.0)
        self.queue_size = int(queue_size)
        self.max_buffer_age_sec = float(max_buffer_age_ms) / 1000.0
        self.rgb_buffer: Deque[BufferedMessage] = deque()
        self.depth_buffer: Deque[BufferedMessage] = deque()
        self.received_rgb_count = 0
        self.received_depth_count = 0
        self.matched_count = 0
        self.dropped_old_rgb_count = 0
        self.dropped_old_depth_count = 0
        self.raw_deltas_ms: List[float] = []
        self.corrected_deltas_ms: List[float] = []
        self._latest_logical_stamp: Optional[float] = None

    def add_rgb(
        self, message: Any, stamp_sec: float, wall_time: float,
    ) -> tuple[List[SyncMatch], List[SyncSample]]:
        self.received_rgb_count += 1
        item = self._item(message, stamp_sec, stamp_sec, wall_time)
        self.rgb_buffer.append(item)
        return self._after_add('rgb', item, wall_time)

    def add_depth(
        self, message: Any, stamp_sec: float, wall_time: float,
    ) -> tuple[List[SyncMatch], List[SyncSample]]:
        self.received_depth_count += 1
        logical_stamp = stamp_sec + self.depth_stamp_offset_ms / 1000.0
        item = self._item(message, stamp_sec, logical_stamp, wall_time)
        self.depth_buffer.append(item)
        return self._after_add('depth', item, wall_time)

    def _item(
        self, message: Any, stamp_sec: float, logical_stamp: float,
        wall_time: float,
    ) -> BufferedMessage:
        values = (stamp_sec, logical_stamp, wall_time)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('message and wall timestamps must be finite')
        if self._latest_logical_stamp is None:
            self._latest_logical_stamp = logical_stamp
        else:
            self._latest_logical_stamp = max(
                self._latest_logical_stamp, logical_stamp)
        return BufferedMessage(message, stamp_sec, logical_stamp, wall_time)

    def _after_add(
        self, kind: str, item: BufferedMessage, wall_time: float,
    ) -> tuple[List[SyncMatch], List[SyncSample]]:
        matches: List[SyncMatch] = []
        samples: List[SyncSample] = []
        opposite = self.depth_buffer if kind == 'rgb' else self.rgb_buffer
        if opposite:
            candidate = min(
                opposite,
                key=lambda other: abs(item.logical_stamp_sec
                                      - other.logical_stamp_sec))
            corrected_delta_ms = self._corrected_delta_ms(
                item if kind == 'rgb' else candidate,
                candidate if kind == 'rgb' else item)
            if abs(corrected_delta_ms) <= self.slop_ms:
                rgb = item if kind == 'rgb' else candidate
                depth = candidate if kind == 'rgb' else item
                self.rgb_buffer.remove(rgb)
                self.depth_buffer.remove(depth)
                raw_delta_ms = (depth.stamp_sec - rgb.stamp_sec) * 1000.0
                match = SyncMatch(
                    rgb=rgb, depth=depth, raw_delta_ms=raw_delta_ms,
                    corrected_delta_ms=corrected_delta_ms)
                matches.append(match)
                self.matched_count += 1
                self.raw_deltas_ms.append(raw_delta_ms)
                self.corrected_deltas_ms.append(corrected_delta_ms)
                samples.append(self._sample(match, wall_time, True, 'matched'))
        samples.extend(self._drop_expired_and_overflow(wall_time))
        return matches, samples

    def _corrected_delta_ms(
        self, rgb: BufferedMessage, depth: BufferedMessage,
    ) -> float:
        return (depth.logical_stamp_sec - rgb.logical_stamp_sec) * 1000.0

    def _drop_expired_and_overflow(self, wall_time: float) -> List[SyncSample]:
        samples: List[SyncSample] = []
        if self._latest_logical_stamp is not None:
            cutoff = self._latest_logical_stamp - self.max_buffer_age_sec
            for kind, buffer in (
                    ('rgb', self.rgb_buffer), ('depth', self.depth_buffer)):
                expired = [item for item in buffer
                           if item.logical_stamp_sec < cutoff]
                for item in expired:
                    buffer.remove(item)
                    samples.append(self._drop_sample(
                        kind, item, wall_time, 'expired'))
        for kind, buffer in (
                ('rgb', self.rgb_buffer), ('depth', self.depth_buffer)):
            while len(buffer) > self.queue_size:
                oldest = min(buffer, key=lambda item: item.arrival_time)
                buffer.remove(oldest)
                samples.append(self._drop_sample(
                    kind, oldest, wall_time, 'queue_overflow'))
        return samples

    def _drop_sample(
        self, kind: str, item: BufferedMessage, wall_time: float, reason: str,
    ) -> SyncSample:
        if kind == 'rgb':
            self.dropped_old_rgb_count += 1
            rgb_stamp, depth_stamp = item.stamp_sec, None
        else:
            self.dropped_old_depth_count += 1
            rgb_stamp, depth_stamp = None, item.stamp_sec
        return SyncSample(
            sync_mode=self.mode, wall_time=wall_time,
            rgb_stamp_sec=rgb_stamp, depth_stamp_sec=depth_stamp,
            raw_delta_ms=None,
            depth_stamp_offset_ms=self.depth_stamp_offset_ms,
            corrected_delta_ms=None, matched=False,
            rgb_buffer_size=len(self.rgb_buffer),
            depth_buffer_size=len(self.depth_buffer), reason=reason)

    def _sample(
        self, match: SyncMatch, wall_time: float, matched: bool, reason: str,
    ) -> SyncSample:
        return SyncSample(
            sync_mode=self.mode, wall_time=wall_time,
            rgb_stamp_sec=match.rgb.stamp_sec,
            depth_stamp_sec=match.depth.stamp_sec,
            raw_delta_ms=match.raw_delta_ms,
            depth_stamp_offset_ms=self.depth_stamp_offset_ms,
            corrected_delta_ms=match.corrected_delta_ms, matched=matched,
            rgb_buffer_size=len(self.rgb_buffer),
            depth_buffer_size=len(self.depth_buffer), reason=reason)

    @property
    def unmatched_rgb_count(self) -> int:
        return self.received_rgb_count - self.matched_count

    @property
    def unmatched_depth_count(self) -> int:
        return self.received_depth_count - self.matched_count

    def summary(self) -> Dict[str, Any]:
        possible = min(self.received_rgb_count, self.received_depth_count)
        absolute_raw = [abs(value) for value in self.raw_deltas_ms]
        absolute_corrected = [abs(value) for value in self.corrected_deltas_ms]
        active = absolute_raw if self.mode == DIRECT_SLOP else absolute_corrected
        result: Dict[str, Any] = {
            'sync_mode': self.mode,
            'received_rgb_count': self.received_rgb_count,
            'received_depth_count': self.received_depth_count,
            'matched_count': self.matched_count,
            'unmatched_rgb_count': self.unmatched_rgb_count,
            'unmatched_depth_count': self.unmatched_depth_count,
            'dropped_old_rgb_count': self.dropped_old_rgb_count,
            'dropped_old_depth_count': self.dropped_old_depth_count,
            'match_rate': self.matched_count / possible if possible else 0.0,
            'mean_abs_delta_ms': statistics.fmean(active) if active else None,
            'median_abs_delta_ms': statistics.median(active) if active else None,
            'p95_abs_delta_ms': _percentile_95(active),
            'max_abs_delta_ms': max(active) if active else None,
            'last_raw_delta_ms': (
                self.raw_deltas_ms[-1] if self.raw_deltas_ms else None),
            'last_corrected_delta_ms': (
                self.corrected_deltas_ms[-1]
                if self.corrected_deltas_ms else None),
            'slop_ms': self.slop_ms,
            'depth_stamp_offset_ms': self.depth_stamp_offset_ms,
        }
        if self.mode == DIRECT_SLOP:
            result['raw_delta_ms'] = result['last_raw_delta_ms']
            result['abs_raw_delta_ms'] = (
                abs(self.raw_deltas_ms[-1]) if self.raw_deltas_ms else None)
            result['mean_abs_raw_delta_ms'] = result['mean_abs_delta_ms']
            result['median_abs_raw_delta_ms'] = result['median_abs_delta_ms']
            result['p95_abs_raw_delta_ms'] = result['p95_abs_delta_ms']
            result['max_abs_raw_delta_ms'] = result['max_abs_delta_ms']
        else:
            result['raw_delta_ms'] = result['last_raw_delta_ms']
            result['corrected_delta_ms'] = result['last_corrected_delta_ms']
            result['abs_corrected_delta_ms'] = (
                abs(self.corrected_deltas_ms[-1])
                if self.corrected_deltas_ms else None)
            result['mean_abs_corrected_delta_ms'] = result['mean_abs_delta_ms']
            result['median_abs_corrected_delta_ms'] = result['median_abs_delta_ms']
            result['p95_abs_corrected_delta_ms'] = result['p95_abs_delta_ms']
            result['max_abs_corrected_delta_ms'] = result['max_abs_delta_ms']
        return result


class OffsetEstimator:
    """Collect bounded nearest-neighbour raw deltas without applying offset."""

    def __init__(self, sample_limit: int, queue_size: int = 30) -> None:
        if sample_limit < 1 or queue_size < 1:
            raise ValueError('estimator limits must be positive')
        self.sample_limit = sample_limit
        self.rgb_stamps: Deque[float] = deque(maxlen=queue_size)
        self.depth_stamps: Deque[float] = deque(maxlen=queue_size)
        self.raw_deltas_ms: List[float] = []

    def add_rgb(self, stamp_sec: float) -> None:
        self._add('rgb', stamp_sec)

    def add_depth(self, stamp_sec: float) -> None:
        self._add('depth', stamp_sec)

    def _add(self, kind: str, stamp_sec: float) -> None:
        if len(self.raw_deltas_ms) >= self.sample_limit:
            return
        own = self.rgb_stamps if kind == 'rgb' else self.depth_stamps
        other = self.depth_stamps if kind == 'rgb' else self.rgb_stamps
        own.append(stamp_sec)
        if not other:
            return
        candidate = min(other, key=lambda value: abs(value - stamp_sec))
        other.remove(candidate)
        own.pop()
        rgb_stamp = stamp_sec if kind == 'rgb' else candidate
        depth_stamp = candidate if kind == 'rgb' else stamp_sec
        self.raw_deltas_ms.append((depth_stamp - rgb_stamp) * 1000.0)

    @property
    def estimated_depth_stamp_offset_ms(self) -> Optional[float]:
        if not self.raw_deltas_ms:
            return None
        return -float(statistics.median(self.raw_deltas_ms))
