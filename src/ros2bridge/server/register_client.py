"""Register Client.

Registering new client.

Methods:
    register_client(socket: Type[WS], client: WS) -> Dict[str, Any]:
        Register new ws client.
"""

from secrets import token_hex
from typing import Any, Dict, Type

import rclpy
from rclpy.node import Node

from ros2bridge.operations.action_client import WSActionClient
from ros2bridge.operations.publisher import WSPublisher
from ros2bridge.operations.service_client import WSServiceClient
from ros2bridge.operations.subscriber import WSSubscriber
from ros2bridge.protocols.ws_server import WSServerProtocol as WS

from tornado.ioloop import PeriodicCallback


def register_client(socket: Type[WS], client: WS) -> Dict[str, Any]:
    """
    Register new ws client.

    Args:
        socket (Type[WS]): WS server.
        client (WS): Connected client.

    Returns:
        Dict[str, Any]: Client info.
    """
    _client_id = f'client_{token_hex(8)}'
    _node = Node(f'ros2bridge_{_client_id}')

    ros_node = PeriodicCallback(
        lambda: rclpy.spin_once(  # type: ignore [no-any-return]
            _node, timeout_sec=0.01
        ),
        1
    )

    ros_node.start()

    _new_client = {
        'client': client,
        'client_id': _client_id,
        'client_node': _node,
        'client_socket': socket,

        'publisher': {},
        'subscriber': {},
        'srv_client': {},
        'action_client': {},
        'terminate': {
            'node': _node.destroy_node,
            'callback': ros_node.stop
        }
    }

    _operations = {
        'publish': WSPublisher(_new_client),
        'subscribe': WSSubscriber(_new_client),
        'srv_client': WSServiceClient(_new_client),
        'action_client': WSActionClient(_new_client)
    }

    _new_client['operations'] = _operations

    return {
        _client_id: _new_client
    }
