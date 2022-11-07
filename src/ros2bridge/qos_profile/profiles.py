"""Profiles.

QoS Profile.

Class:
    QoSReliabilityPolicy(Enum):
        QoS policy for reliability.

    QoSDurabilityPolicy(Enum):
        QoS policy for durability.

    QoSHistoryPolicy(Enum):
        QoS policy for history.

Methods:
    process_qos_profile(qos: Dict[str, str]) -> QoSProfile:
        Create QoS profile.
"""
import contextlib
from enum import Enum
from typing import Dict

from rclpy.qos import QoSProfile


class QoSReliabilityPolicy(Enum):
    """QoS policy for reliability."""

    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2
    UNKNOWN = 3


class QoSDurabilityPolicy(Enum):
    """QoS policy for durability."""

    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2
    UNKNOWN = 3


class QoSHistoryPolicy(Enum):
    """QoS policy for history."""

    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2
    UNKNOWN = 3


def process_qos_profile(qos: Dict[str, str]) -> QoSProfile:
    """Create QoS profile.

    Args:
        qos (Dict[str, str]): User requested QoS.

    Returns:
        QoSProfile: Created profile.
    """
    _profile = QoSProfile(history=0)

    if _qos := qos.get('reliability'):
        with contextlib.suppress(AttributeError):
            _reliability = getattr(QoSReliabilityPolicy, _qos.upper()).value
            _profile.reliability = _reliability

    if _qos := qos.get('durability'):
        with contextlib.suppress(AttributeError):
            _durability = getattr(QoSDurabilityPolicy, _qos.upper()).value
            _profile.durability = _durability

    if _qos := qos.get('history'):
        with contextlib.suppress(AttributeError):
            _history = getattr(QoSHistoryPolicy, _qos.upper()).value
            # if _history == 1:
            #     _depth = qos['depth']
            _profile.history = _history
            # _profile.depth = _depth

    return _profile
