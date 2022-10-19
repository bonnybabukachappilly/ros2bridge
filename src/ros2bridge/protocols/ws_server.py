"""
Blueprint of WS Server.

WSServer:
    WS Server.
"""

from typing import Protocol


class WSServer(Protocol):
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
    on_open(self) -> None:
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

    def on_open(self) -> None:
        """Handle new client connection."""
        ...

    def on_message(self, message: str) -> None:
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
        ...

    def set_default_headers(self) -> None:
        """CROS Headers."""
        ...
