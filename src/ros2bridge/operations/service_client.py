"""ROS Service Client.

This implementation ROS Service Client functionalities.

Class:
    WSServiceClient:
        Create ros service client.
"""

import json
from dataclasses import dataclass
from typing import Any, Dict

import rclpy
from rclpy.node import Node

from ros2bridge.protocols.ws_server import WSServerProtocol
from ros2bridge.utils.data_parser import RosDataParser, RosDataType


@dataclass
class WSServiceClient:
    """ROS Service Client.

    Create ros service client.

    Attributes:
        client: Dict[str, Any]
        data_parser: RosDataParser

    Methods:
        handle_operation(self, data: Dict[str, Any]) -> None:
            Create and call ROS service client on client request.
    """

    client: Dict[str, Any]
    data_parser = RosDataParser(data_type=RosDataType.SERVICE)

    def handle_operation(self, data: Dict[str, Any]) -> None:
        """Create and call ROS service client on client request.

        Args:
            data (Dict): Request from ws client.
        """
        _client_name: str = self.client['client_id']
        _client: WSServerProtocol = self.client['client']
        self._node: Node = self.client['client_node']

        srv_name = data['srv_name']
        srv_type = data['srv_type']
        action = data['action']

        if action == 'create':
            print(
                f'Client: {_client_name} created a service client. | ' +
                f'Service Name: {srv_name} | Type: {srv_type}'
            )

            _srv_type = self.data_parser.import_type(srv_type)
            srv_cli = self._node.create_client(
                srv_type=_srv_type,
                srv_name=srv_name
            )

            self.client['srv_client'][srv_name] = {
                'client': srv_cli,
                'srv_type': srv_type
            }

        elif action == 'call':
            response = data.copy()

            message = data['message']

            srv_cli = self.client['srv_client'][srv_name]

            if srv_cli['srv_type'] != data['srv_type']:
                response['message'] = 'Service type mismatch, please check.'
                print(response['message'])
                _client.send_message(json.dumps(response))
                return

            while not srv_cli['client'].wait_for_service(timeout_sec=1.0):
                response['message'] = f'Waiting for service {srv_name}'
                _client.send_message(json.dumps(response))

            request = self.data_parser.pack_data_to_ros(
                data=message,
                module=self.data_parser.get_module_instance(
                    module=srv_type
                )
            )

            future = srv_cli['client'].call_async(request)

            rclpy.spin_until_future_complete(
                node=self._node,
                future=future
            )

            result = future.result()

            msg = self.data_parser.pack_data_to_json(
                module=result,
                output={}
            )

            data['message'] = msg

            _client.send_message(json.dumps(data))
