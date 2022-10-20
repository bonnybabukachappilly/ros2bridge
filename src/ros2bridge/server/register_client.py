"""
Registering new client.

register_client: Register new WS client
"""

from secrets import token_hex
from typing import Any, Dict, Type

from rclpy.node import Node

from ros2bridge.operations.publisher import WSPublisher
from ros2bridge.protocols.ws_server import WSServerProtocol as WS


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
    _node = Node(_client_id)

    _operations = {
        'publish': WSPublisher(socket, client),
        # 'subscribe': None,
        # 'srv_client': None,
        # 'action_client': None
    }

    return {
        _client_id: {
            'client': client,
            'client_id': _client_id,
            'client_node': _node,
            'client_socket': socket,

            'operations': _operations,
            'publisher': {},
            'subscriber': {},
            'srv_client': {},
            'action_client': {},
            'terminate': _node.destroy_node
        }
    }
