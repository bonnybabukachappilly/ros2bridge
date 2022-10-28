"""ROS Subscriber.

This implementation ROS Subscriber functionalities.

Class:
    WSPublisher:
        Create ros Subscriber and send message to the client.
"""

import json
from dataclasses import dataclass
from functools import partial
from typing import Any, Dict

from rclpy.node import Node

from ros2bridge.protocols.ws_server import WSServerProtocol
from ros2bridge.utils.data_parser import RosDataParser, RosDataType


@dataclass
class WSSubscriber:
    """ROS Subscriber.

    Create ros Subscriber and send message to the client.

    Attributes:
        client: Dict[str, Any]
        data_parser: RosDataParser

    Methods:
        handle_operation(self, data: Dict[str, Any]) -> None:
            Create and subscribe to ROS message on client request.

        subscription_callback(self, data: Dict[str, Any], msg: Any) -> None:
            Handle callback from subscriber and send it to the client.

        check_topic(self, topic_name: str) -> bool:
            Check if the given topic is already published.
    """

    client: Dict[str, Any]
    data_parser = RosDataParser(data_type=RosDataType.MESSAGE)

    def handle_operation(self, data: Dict[str, Any]) -> None:
        """Create and subscribe to ROS message on client request.

        Args:
            data (Dict): Request from ws client.
        """
        _client_name: str = self.client['client_id']
        self._client: WSServerProtocol = self.client['client']
        self._node: Node = self.client['client_node']

        topic_name = data['topic']
        message_type = data['type']

        if (_my_client := self.client['subscriber'].get(topic_name)):
            if data.get('unsubscribe'):
                data['message'] = f'unsubscribing from Topic: {topic_name}'
                _my_client['subscriber'].destroy()
            else:
                data['message'] = f'Already subscribing to {topic_name}'

            print(data['message'])
            self._client.send_message(json.dumps(data))

            return

        print(
            f'Client: {_client_name} created a subscriber. | ' +
            f'Topic: {topic_name} | Type: {message_type}'
        )

        if not self.check_topic(topic_name):
            data['message'] = 'Topic not yet published'
            print(data['message'])
            self._client.send_message(json.dumps(data))

        ros_msg_type = self.data_parser.import_type(
            package=message_type
        )

        partial_callback = partial(self.subscription_callback, data)

        # TODO: ADD QOS PROFILES
        # _qos = get_qos_profile(_qos) if (_qos := data.get('qos')) else 10
        _qos = 10

        subscriber = self._node.create_subscription(
            ros_msg_type,
            topic_name,
            partial_callback,
            _qos
        )
        subscriber.topic_name

        self.client['subscriber'][topic_name] = {
            'subscriber': subscriber,
            'message_type': data['type']
        }

    def subscription_callback(self, data: Dict[str, Any], msg: Any) -> None:
        """Handle callback from subscriber and send it to the client.

        Args:
            data (Dict[str, Any]): Additional fields to represent msg info.
            msg (Any): msg from subscribed publisher.
        """
        message = self.data_parser.pack_data_to_json(
            module=msg,
            output={}
        )

        data['message'] = message

        self._client.send_message(json.dumps(data))

    def check_topic(self, topic_name: str) -> bool:
        """Check if the given topic is already published.

        Args:
            topic_name (str): Name of the topic to publish to.

        Returns:
            bool: Status of the topic, True: exists else no publisher.
        """
        return bool(self._node.get_publishers_info_by_topic(topic_name))
