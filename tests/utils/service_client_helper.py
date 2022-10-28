"""Helper for service testing."""

import json
from time import sleep
from websocket._core import WebSocket


class ServiceTurtleBot:
    """_summary_."""

    def __init__(self, srv_name, srv_type, message) -> None:
        """_summary_.

        Args:
            srv_name (_type_): _description_
            srv_type (_type_): _description_
            message (_type_): _description_
        """
        self._name = srv_name
        self._type = srv_type
        self._message = message

        self._pre_message = None

    @property
    def set_websocket(self) -> WebSocket:
        """_summary_.

        Returns:
            WebSocket: _description_
        """
        return self._websocket

    @set_websocket.setter
    def set_websocket(self, websocket: WebSocket) -> None:
        """_summary_.

        Args:
            websocket (WebSocket): _description_
        """
        self._websocket = websocket

    def ws_srv_client(self, call=False) -> None:
        """Send a message via ws for testing.

        Args:
            call (bool, optional): Call service. Defaults to False.
        """
        service_create = {
            'operation': 'srv_client',
            'action': 'create',
            'srv_name': self._name,
            'srv_type': self._type,
        }

        service_call = {
            'operation': 'srv_client',
            'action': 'call',
            'srv_name': self._name,
            'srv_type': self._type,
            'message': self._message,
        }

        if call:
            self._websocket.send(json.dumps(service_call))
        self._websocket.send(json.dumps(service_create))

    def create_call_service(self) -> None:
        """_summary_."""
        self.ws_srv_client()
        sleep(2)
        self.ws_srv_client(call=True)

    def get_message(self, old_data=False, keys: list = None) -> dict:
        """Get message from websocket for testing.

        Args:
            old_data (bool, optional): previous data. Defaults to False.
            keys (list, optional): Filter data based on key. Defaults to None.

        Returns:
            dict: Message received from websocket.
        """
        if old_data and self._pre_message:
            if keys and isinstance(keys, list):
                return self.get_by_key(self._pre_message, keys)
            return self._pre_message

        _count = 10

        while True:
            _recv: dict = json.loads(self._websocket.recv())
            _count -= 1

            if _recv.get('srv_name') == self._name:
                break

            if _count < 0:
                assert False
            sleep(1)

        self._pre_message = _recv.get('message')

        if keys and isinstance(keys, list):
            return self.get_by_key(self._pre_message, keys)

        return self._pre_message

    def get_by_key(self, data: dict, keys: list) -> dict:
        """Filter the data by given key.

        Args:
            data (dict): Data to filter.
            keys (list): Keys for filtering.

        Returns:
            dict: filtered data.
        """
        for key in keys:
            data = data.get(key)

        return data
