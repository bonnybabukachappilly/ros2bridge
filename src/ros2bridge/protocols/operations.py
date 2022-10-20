"""
Blueprint of ROS Operation.

RosOperationsProtocol:
    Create ROS Operation.
"""

from dataclasses import dataclass
from typing import Any, Dict, Protocol, Type

from ros2bridge.protocols.ws_server import WSServerProtocol


@dataclass
class RosOperationsProtocol(Protocol):
    """
    Model class for creating different ros operations.

    Attributes
    ----------
    socket: Type[WSServerProtocol]
        WSServerProtocol class.

    client: WSServerProtocol
        WSServerProtocol instance.

    Method:
    -------
    handle_operation(self, data: Dict) -> None
        Run ws client based on request.
    """

    socket: Type[WSServerProtocol]
    client: WSServerProtocol

    def handle_operation(self, data: Dict[str, Any]) -> None:
        """Run ws client based on request.

        Args:
            data (Dict): Request from ws client.
        """
        ...
