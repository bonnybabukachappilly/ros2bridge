"""Route client message.

This module routes all client request to corresponding operations.

Methods:
    get_operation(
        client: Dict[str, Any], operation: str
    ) -> RosOperationsProtocol:
        Get client requested operation.

    route_message(client: Dict[str, Any], message: str) -> None:
        Route client request.
"""
import json
from json.decoder import JSONDecodeError
from typing import Any, Dict

from ros2bridge.bridge_exceptions.operation_exception import (
    OperationNotFoundException
)
from ros2bridge.protocols.operations import RosOperationsProtocol as RO


def get_operation(client: Dict[str, Any], operation: str) -> RO:
    """Get client requested operation.

    Args:
        client (Dict[str, Any]): Websocket client.
        operation (str): Requested operation

    Raises:
        OperationNotFoundException: Requested operation not found.

    Returns:
        RO: Requested operation.
    """
    _response = f'{operation} is not supported yet. ' \
        'Check key or please be patient while we add '\
        'this feature in next update.'

    try:
        _operation: RO = client['operations'][operation]

        if not _operation:
            raise KeyError

    except KeyError as e:
        client['client'].send_message(_response)
        raise OperationNotFoundException(_response) from e

    return _operation


def route_message(client: Dict[str, Any], message: str) -> None:
    """Route client request.

    Args:
        client (Dict[str, Any]): WS Client.
        message (str): Client request.
    """
    try:
        _msg = json.loads(message)

        _operation: RO = get_operation(
            client=client,
            operation=_msg['operation']
        )

        _operation.handle_operation(_msg)

    except OperationNotFoundException as e:
        print(e)

    except JSONDecodeError:
        msg = 'Unsupported request. Please check the request syntax.'
        ws_client = client.get('client')  # type: ignore [assignment]
        ws_client.send_message(  # type: ignore [union-attr]
            json.dumps({'ERROR': msg})
        )
        print(msg)
