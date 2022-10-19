"""
Registering new client.

register_client: Register new WS client
"""

from secrets import token_hex
from typing import Any, Dict, Type

from rclpy.node import Node

from ros2bridge.server.ws_server import WebSocketServer as WSS


def register_client(socket: Type[WSS], client: WSS) -> Dict[str, Any]:
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
        'publish': None,
        'subscribe': None,
        'srv_client': None,
        'action_client': None
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
            'terminate': []
        }
    }
