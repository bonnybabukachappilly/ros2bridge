"""
Exception occurred in websocket connection.

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
