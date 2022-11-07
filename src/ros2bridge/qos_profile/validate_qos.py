"""Validate QoS.

Validate QoS requested by the user.

Class:
    ValidateQos:
        Create new websocket server.
"""
from typing import Any, Dict

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.qos import QoSProfile as QOS


class ValidateQos:
    """Validate QoS.

    Validate user requested QoS profile wether it is supported.

    Methods:
        validate_reliability(self) -> bool:
            Check reliability combination is possible.

        validate_durability(self) -> bool:
            Check durability combination is possible.

        validate(self, usr_qos: QOS, sys_qos: QOS) -> Dict[str, Any] or bool:
            Validate user requested QoS profile wether it is supported.
    """

    def validate_reliability(
        self
    ) -> bool or Any:  # type: ignore [valid-type]
        """Check reliability combination is possible.

        Returns:
            bool or Any: Possible or not.
        """
        if self._system_qos.reliability == ReliabilityPolicy.BEST_EFFORT:
            return self._user_qos.reliability == ReliabilityPolicy.BEST_EFFORT

        elif self._system_qos.reliability == ReliabilityPolicy.RELIABLE:
            return self._user_qos.reliability in [
                ReliabilityPolicy.BEST_EFFORT,
                ReliabilityPolicy.RELIABLE
            ]
        return False

    def validate_durability(
        self
    ) -> bool or Any:  # type: ignore [valid-type]
        """Check durability combination is possible.

        Returns:
            bool or Any: Possible or not.
        """
        if self._system_qos.durability == DurabilityPolicy.VOLATILE:
            return self._user_qos.durability == DurabilityPolicy.VOLATILE

        elif self._system_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL:
            return self._user_qos.durability in [
                DurabilityPolicy.VOLATILE,
                DurabilityPolicy.TRANSIENT_LOCAL
            ]

        return False

    def validate(
            self, usr_qos: QOS, sys_qos: QOS
    ) -> Dict[str, Any] or bool:  # type: ignore [valid-type]
        """Validate user requested QoS profile wether it is supported.

        Args:
            usr_qos (QOS): User requested QoS.
            sys_qos (QOS): created ROS operations QoS.

        Returns:
            Dict[str, Any] or bool: Error message or any error in combination.
        """
        self._user_qos = usr_qos
        self._system_qos = sys_qos

        errors = {}

        if not self.validate_reliability():
            errors['qos_reliability'] = {
                'Publisher QoS': self._system_qos.reliability,
                'Requested QoS': self._user_qos.reliability,
                'message': 'Combination not possible.'
            }

        if not self.validate_durability():
            errors['qos_durability'] = {
                'Publisher QoS': self._system_qos.durability,
                'Requested QoS': self._user_qos.durability,
                'message': 'Combination not possible.'
            }

        # Returning 'False' means no error.
        return errors or False
