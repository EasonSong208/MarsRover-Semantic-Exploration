"""Pure, ROS-independent policy helpers for M1 preflight checks."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable, Sequence


class Severity(IntEnum):
    """Ordering used to compute the overall readiness result."""

    PASS = 0
    WARN = 1
    FAIL = 2


@dataclass(frozen=True)
class CheckResult:
    """One human-readable preflight observation."""

    severity: Severity
    name: str
    detail: str

    @property
    def label(self) -> str:
        """Return a stable display label."""
        return self.severity.name


def check_topic(
    topic: str,
    actual_types: Sequence[str],
    expected_type: str,
    publisher_names: Sequence[str],
    expected_publishers: int | None = None,
) -> CheckResult:
    """Check topic type and, when requested, exact publisher count."""
    if not actual_types:
        return CheckResult(Severity.FAIL, topic, 'topic is missing')
    if expected_type not in actual_types:
        return CheckResult(
            Severity.FAIL,
            topic,
            f'expected {expected_type}; found {", ".join(actual_types)}',
        )
    if expected_publishers is not None and len(publisher_names) != expected_publishers:
        names = ', '.join(publisher_names) if publisher_names else 'none'
        return CheckResult(
            Severity.FAIL,
            topic,
            f'expected {expected_publishers} publisher(s); '
            f'found {len(publisher_names)}: {names}',
        )
    names = ', '.join(publisher_names) if publisher_names else 'none'
    return CheckResult(
        Severity.PASS,
        topic,
        f'type={expected_type}; publishers={names}',
    )


def check_unique_node(node_name: str, matches: Sequence[str]) -> CheckResult:
    """Require exactly one fully-qualified node with the requested basename."""
    if len(matches) == 1:
        return CheckResult(Severity.PASS, node_name, matches[0])
    return CheckResult(
        Severity.FAIL,
        node_name,
        f'expected one node; found {len(matches)}: '
        f'{", ".join(matches) if matches else "none"}',
    )


def warn_publishers(topic: str, publisher_names: Sequence[str]) -> CheckResult:
    """Expose direct-control endpoints without claiming that they are active."""
    if not publisher_names:
        return CheckResult(Severity.PASS, topic, 'no direct publisher endpoints')
    return CheckResult(
        Severity.WARN,
        topic,
        'publisher endpoints require operator review; graph presence does not '
        f'prove active commands: {", ".join(publisher_names)}',
    )


def summarize(results: Iterable[CheckResult]) -> tuple[Severity, str]:
    """Return the highest severity and a motion-readiness summary."""
    results = tuple(results)
    highest = max((result.severity for result in results), default=Severity.FAIL)
    if highest == Severity.FAIL:
        return highest, 'NOT READY FOR MOTION'
    if highest == Severity.WARN:
        return highest, 'READY FOR OPERATOR REVIEW; NOT AUTHORIZED TO MOVE'
    return highest, 'PREFLIGHT PASSED; MOTION STILL REQUIRES USER CONFIRMATION'
