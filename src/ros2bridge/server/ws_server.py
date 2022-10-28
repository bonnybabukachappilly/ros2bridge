"""Websocket Server.

Websocket side of the ros2bridge.

Class:
    WebSocketServer(WebSocketHandler):
        Create new websocket server.
"""
from typing import Any, Dict

from ros2bridge.bridge_exceptions.websocket_exception import (
    ClientNotFoundException,
    MessageTypeException,
)
from ros2bridge.protocols.ws_server import WSServerProtocol
from ros2bridge.server.register_client import register_client
from ros2bridge.server.route_message import route_message

from tornado.websocket import (
    WebSocketClosedError,
    WebSocketHandler,
)


class WebSocketServer(WebSocketHandler):
    """
    Create new websocket server.

    Create a websocket server using tornado.
    Handles all incoming and outgoing clients and requests.

    Attributes:
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
        cls = self.__class__

        _client = cls.get_client_by_object(self)
        _client = cls.connected_clients.get(_client)

        route_message(_client, message)  # type: ignore [arg-type]

    def on_close(self) -> None:
        """Client disconnected."""
        cls = self.__class__

        _client = cls.get_client_by_object(self)
        _terminate = cls.connected_clients.get(
            _client
        ).get('terminate')  # type: ignore [union-attr]

        _terminate.get('node')()
        _terminate.get('callback')()

        print(f'ROS Node: {_client} terminated.')

        del cls.connected_clients[_client]

        print(f'Client: {_client} disconnected.')

    def send_message(self, message: str) -> None:
        """Send given message to client.

        Args:
            message (str): Message to send.
        """
        try:
            self.write_message(message)
        except WebSocketClosedError:
            cls = self.__class__
            _client = cls.get_client_by_object(self)

            print(f'Client: {_client} has already closed.')
            self.on_close()
        except MessageTypeException:
            print(f'{type(message)} is not supported.')

    @classmethod
    def get_client_by_object(cls, client_obj: WSServerProtocol) -> object:
        """Get client by giving an instance.

        Args:
            client_obj(WebSocketServer): Required client object.

        Raises:
            ClientNotFoundException: Given client not found.

        Returns:
            object: Return client.
        """
        for clients in cls.connected_clients:
            client = cls.connected_clients.get(clients)

            if client.get('client') == client_obj:  # type: ignore [union-attr]
                return clients

        raise ClientNotFoundException

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
