"""ROS Publisher.

This implementation ROS Publisher functionalities.

Class:
    WSPublisher:
        Create ros publisher and publish client message.
"""

import json
from dataclasses import dataclass
from typing import Any, Dict

from rclpy.node import Node
from rclpy.publisher import Publisher

from ros2bridge.protocols.ws_server import WSServerProtocol
from ros2bridge.utils.data_parser import RosDataParser, RosDataType


@dataclass
class WSPublisher:
    """ROS Publisher.

    Create ros publisher and publish client message.

    Attributes:
        client: Dict[str, Any]
        data_parser: RosDataParser

    Methods:
        handle_operation(self, data: Dict[str, Any]) -> None:
            Create and publish ROS message on client request.

        check_topic(self, topic_name: str) -> bool:
            Check if the given topic is already published.
    """

    client: Dict[str, Any]

    data_parser = RosDataParser(data_type=RosDataType.MESSAGE)

    def handle_operation(self, data: Dict[str, Any]) -> None:
        """Create and publish ROS message on client request.

        Args:
            data (Dict): Request from ws client.
        """
        _client_name: str = self.client['client_id']
        _client: WSServerProtocol = self.client['client']
        self._node: Node = self.client['client_node']

        topic_name = data['topic']
        message_type = data['type']
        message = data['message']

        if not self.check_topic(topic_name):
            ros_msg_type = self.data_parser.import_type(
                package=message_type
            )

            publisher = self._node.create_publisher(
                ros_msg_type,
                topic_name,
                10
            )

            self.client['publisher'][topic_name] = {
                'publisher': publisher,
                'message_type': message_type
            }

            print(
                f'Client: {_client_name} created a publisher. | ' +
                f'Topic: {topic_name} | Type: {message_type}'
            )

        _client_publisher = self.client['publisher'][topic_name]

        if _client_publisher['message_type'] != message_type:
            message = 'Topic type mismatch, please check.'
            data['message'] = message
            _client.send_message(json.dumps(data))

        else:
            msg = self.data_parser.pack_data_to_ros(
                data=message,
                module=self.data_parser.get_module_instance(
                    module=message_type
                )
            )

            try:
                _publisher: Publisher = _client_publisher['publisher']
                _publisher.publish(msg)
            except AttributeError as e:
                data['message'] = str(e)
                _client.send_message(json.dumps(data))

    def check_topic(self, topic_name: str) -> bool:
        """Check if the given topic is already published.

        Args:
            topic_name (str): Name of the topic to publish to.

        Returns:
            bool: Status of the topic, True: exists else no publisher.
        """
        return bool(self._node.get_publishers_info_by_topic(topic_name))
