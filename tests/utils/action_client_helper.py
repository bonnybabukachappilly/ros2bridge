"""Helper for Action testing."""

import json
from time import sleep

from websocket._core import WebSocket


class ActionTurtleBot:
    """_summary_."""

    def __init__(self, action_name, action_type, message) -> None:
        """_summary_.

        Args:
            action_name (_type_): _description_
            action_type (_type_): _description_
            message (_type_): _description_
        """
        self._name = action_name
        self._type = action_type
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

    def ws_action_client(self, call=False) -> None:
        """Send a message via ws for testing.

        Args:
            call (bool, optional): Call Action. Defaults to False.
        """
        action_create = {
            'operation': 'action_client',
            'action': 'create',
            'action_name': self._name,
            'action_type': self._type,
        }

        action_call = {
            'operation': 'action_client',
            'action': 'call',
            'action_name': self._name,
            'action_type': self._type,
            'message': self._message,
        }

        if call:
            self._websocket.send(json.dumps(action_call))
        self._websocket.send(json.dumps(action_create))

    def create_call_service(self) -> None:
        """_summary_."""
        self.ws_action_client()
        sleep(2)
        self.ws_action_client(call=True)

    def get_message(self, old_data=False, keys: list = None) -> dict:
        """Get message from websocket for testing.

        Args:
            old_data (bool, optional): previous data.Defaults to False.
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

            if _recv.get('action_name') == self._name:
                break

            if _count < 0:
                assert False
            sleep(1)

        self._pre_message = _recv

        if keys and isinstance(keys, list):
            return self.get_by_key(self._pre_message, keys)

        return self._pre_message

    def collect_action_response(self) -> None:
        """_summary_."""
        self.response = []
        self.feedback = []
        self.result = []

        while True:
            msg = self.get_message()
            if msg.get('action_response') == 'response':
                self.response.append(msg)
            elif msg.get('action_response') == 'feedback':
                self.feedback.append(msg)
            elif msg.get('action_response') == 'result':
                self.result.append(msg)
                break

    def get_action_response(self, key: str) -> list:
        """_summary_.

        Args:
            key (str): _description_

        Returns:
            list: _description_
        """
        if key == 'response':
            return self.response
        elif key == 'feedback':
            return self.feedback
        elif key == 'result':
            return self.result

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

    def check_keys(self, keys: list, msg: dict) -> None:
        """Test for given keys in message.

        Args:
            keys (list): Keys to check.
            msg (dict): Message to check for keys.
        """
        for key in keys:
            assert msg.get(key) is not None
