"""QoS Profile.

Handle user request for QoS.

Class:
    QosProfile:
        Create user requested qos profile.
"""

from dataclasses import dataclass
from typing import Any, Dict

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.topic_endpoint_info import TopicEndpointInfo as TE_Info

from ros2bridge.qos_profile.profiles import process_qos_profile
from ros2bridge.qos_profile.validate_qos import ValidateQos


@dataclass
class QosProfile:
    """QoSProfile.

    Create user requested qos profile.

    Attributes:
        client: Dict[str, Any]

    Methods:
        handle_profile(self, message: Dict[str, Any]) -> QoSProfile or bool:
            Handle user requested QoS.

        subscriber(self, data: Dict[str, Any]) -> QoSProfile or bool:
            Create QoS profile for subscriber.
    """

    client: Dict[str, Any]
    validate = ValidateQos()

    def handle_profile(
        self, message: Dict[str, Any]
    ) -> QoSProfile or bool:  # type: ignore [valid-type]
        """Handle user requested QoS.

        Args:
            message (Dict[str, Any]): User request.

        Returns:
            QoSProfile or bool: QoS profile.
        """
        if not message.get('qos'):
            return True

        if message['operation'] == 'subscribe':
            return self.subscriber(message)  # type: ignore [valid-type]

    def subscriber(
        self, data: Dict[str, Any]
    ) -> QoSProfile or bool:  # type: ignore [valid-type]
        """Create QoS profile for subscriber.

        Args:
            data (Dict[str, Any]): User request.

        Returns:
            QoSProfile or bool: QoS Profile.
        """
        self._node: Node = self.client['client_node']

        _topic_name = data['topic']

        publisher: TE_Info = self._node.get_publishers_info_by_topic(
            topic_name=_topic_name
        )[0]

        _sys_qos = publisher._qos_profile
        _usr_qos = process_qos_profile(data['qos'])

        if (_response := self.validate.validate(_usr_qos, _sys_qos)):
            self.client['client'].send_message(_response)
            return False  # Verification failed
        else:
            return _usr_qos  # All OK
