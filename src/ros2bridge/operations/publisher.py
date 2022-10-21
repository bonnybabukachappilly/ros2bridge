"""
Blueprint of ROS Operation.

RosOperationsProtocol:
    Create ROS Operation.
"""

import json
from dataclasses import dataclass
from typing import Any, Dict, Type

from rclpy.node import Node
from rclpy.publisher import Publisher

from ros2bridge.protocols.ws_server import WSServerProtocol
from ros2bridge.utils.data_parser import RosDataParser, RosDataType


@dataclass
class WSPublisher:
    """
    Model class for creating different ros operations.

    Attributes
    ----------
    socket: Type[WSServerProtocol]
        WSServerProtocol class.

    client: WSServerProtocol
        WSServerProtocol instance.

    Method:
    -------
    handle_operation(self, data: Dict) -> None
        Run ws client based on request.
    """

    socket: Type[WSServerProtocol]
    client: WSServerProtocol

    data_parser = RosDataParser(data_type=RosDataType.MESSAGE)

    def handle_operation(self, data: Dict[str, Any]) -> None:
        """Run ws client based on request.

        Args:
            data (Dict): Request from ws client.
        """
        _client_name = self.socket.get_client_by_object(self.client)
        _client = self.socket.connected_clients[_client_name]

        self._node: Node = _client['client_node']

        topic_name = data['topic']
        user_message_type = data['type']
        message = data['message']

        # set_trace()
        if not self.check_topic(topic_name):
            message_type = self.data_parser.import_type(
                package=user_message_type
            )

            publisher = self._node.create_publisher(
                message_type,
                topic_name,
                10
            )

            _client['publisher'][topic_name] = {
                'publisher': publisher,
                'message_type': user_message_type
            }

        _client_publisher = _client['publisher'][topic_name]

        if _client_publisher['message_type'] != user_message_type:
            message = 'Topic type mismatch, please check.'
            data['message'] = message
            self.client.send_message(json.dumps(data))

        else:
            msg = self.data_parser.pack_data_to_ros(
                data=message,
                module=self.data_parser.get_module_instance(
                    module=user_message_type
                )
            )

            try:
                _publisher: Publisher = _client_publisher['publisher']
                _publisher.publish(msg)
            except AttributeError as e:
                data['message'] = str(e)
                self.client.send_message(json.dumps(data))

        # set_trace()

    def check_topic(self, topic_name: str) -> bool:
        """Check if the given topic is already published.

        Args:
            topic_name (str): Name of the topic to publish to.

        Returns:
            bool: Status of the topic, True: exists else no publisher.
        """
        return bool(self._node.get_publishers_info_by_topic(topic_name))
