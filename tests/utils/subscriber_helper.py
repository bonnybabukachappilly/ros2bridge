"""Helper for subscriber testing."""

import json
from time import sleep

from websocket._core import WebSocket


class SubscribeTurtleBot:
    """_summary_."""

    def __init__(self, msg_topic, msg_type) -> None:
        """_summary_.

        Args:
            msg_topic (_type_): _description_
            msg_type (_type_): _description_
        """
        self._topic = msg_topic
        self._type = msg_type

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

    def ws_subscribe(self, deactivate=False) -> None:
        """_summary_.

        Args:
            deactivate (bool, optional): _description_. Defaults to False.
        """
        subscriber = {
            'operation': 'subscribe',
            'topic': self._topic,
            'type': self._type,
        }

        if deactivate:
            subscriber['unsubscribe'] = True

        self._websocket.send(json.dumps(subscriber))

    def get_message(self, old_data=False, keys: list = None) -> dict:
        """Get message from websocket for testing.

        Args:
            old_data (bool, optional): Send previously fetched data.. Defaults to False.
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

            if _recv.get('topic') == self._topic:
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
