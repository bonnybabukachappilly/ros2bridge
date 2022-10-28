"""Websocket Exception.

This file contains custom exception for Websocket connection.

Class:
    ClientNotFoundException:
        The client cannot be found.

    MessageTypeException:
        Message type of websocket does not match.
"""


class ClientNotFoundException(Exception):
    """The client cannot be found."""

    ...


class MessageTypeException(Exception):
    """Message type of websocket does not match."""

    ...
