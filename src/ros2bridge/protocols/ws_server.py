"""Websocket Server.

Blueprint of WS Server.

WebSocketServer:
    Create new websocket server.
"""


from typing import Any, Dict, Protocol


class WSServerProtocol(Protocol):
    """
    Create new websocket server.

    Create a websocket server using tornado.
    Handles all incoming and outgoing clients and requests.

    Attributes
        connected_clients: Dict[object, Any]
            List of client connected to the server.

    Methods:
        open(self) -> None:
            Handle new client connection.

        on_message(self, message: str) -> None:
            Message received from client.

        on_close(self) -> None:
            Client disconnected.

        send_message(self, message: str) -> None
            Send given message to client.

        @classmethod
        def get_client_by_object(cls, client_obj: 'WebSocketServer') -> object:
            Get client by giving an instance.

        check_origin(self, _: str) -> bool:
            Allow CROS.

        set_default_headers(self) -> None:
            CROS Headers.
    """

    connected_clients: Dict[object, Any] = {}

    def open(self) -> None:  # type: ignore [override] # noqa: A003
        """Handle new client connection."""
        ...

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

    @classmethod
    def get_client_by_object(cls, client_obj: 'WSServerProtocol') -> object:
        """Get client by giving an instance.

        Args:
            client_obj(WSServerProtocol): Required client object.

        Raises:
            ClientNotFoundException: Given client not found.

        Returns:
            object: Return client.
        """
        ...

    def check_origin(self, _: str) -> bool:
        """Allow CROS.

        Args:
            _ (str): Origin

        Returns:
            bool: True, Accept all origin.
        """
        ...

    def set_default_headers(self) -> None:
        """CROS Headers."""
        ...
