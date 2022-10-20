"""
Implementation of WS Server.

WebSocketServer:
    Create WS Server.
"""

from typing import Any, Dict

from ros2bridge.server.register_client import register_client

from tornado.websocket import WebSocketHandler


class WebSocketServer(WebSocketHandler):
    """
    Create new websocket server.

    Create a websocket server using tornado.
    Handles all incoming and outgoing clients and requests.

    Attributes
    ----------
    connected_clients: Dict[object, Any]
        List of client connected to the server.

    Methods
    -------
    open(self) -> None:
        Handle new client connection.

    on_message(self, message: str) -> None:
        Message received from client.
    on_close(self) -> None:
        Client disconnected.

    send_message(self, message: str) -> None
        Send given message to client.

    check_origin(self, _: str) -> bool:
        Allow CROS.

    set_default_headers(self) -> None:
        CROS Headers.
    """

    connected_clients: Dict[object, Any] = {}

    def open(self) -> None:  # type: ignore [override] # noqa: A003
        """Handle new client connection."""
        cls = self.__class__

        # Register new client
        _client = register_client(
            socket=cls,  # type: ignore [arg-type]
            client=self  # type: ignore [arg-type]
        )
        _client_name = list(_client.keys())[0]
        print(f'Client connected: {_client_name}')

        # Update client list
        cls.connected_clients[_client_name] = _client[_client_name]

    def on_message(self, message: str) -> None:  # type: ignore [override]
        """Message received from client.

        Args:
            message (str): Message received from client.
        """
        ...

    def on_close(self) -> None:
        """Client disconnected."""
        ...

    def send_message(self, message: str) -> None:
        """Send given message to client.

        Args:
            message (str): Message to send.
        """
        ...

    def check_origin(self, _: str) -> bool:
        """Allow CROS.

        Args:
            _ (str): Origin

        Returns:
            bool: True, Accept all origin.
        """
        return True

    def set_default_headers(self) -> None:
        """CROS Headers."""
        self.set_header('Content-Type', 'application/json')
        self.set_header('Access-Control-Allow-Origin', '*')
        self.set_header('Access-Control-Allow-Headers', 'x-requested-with')
        super().set_default_headers()
