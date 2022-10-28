"""ROS Operations.

Blueprint of ROS Operation.

RosOperationsProtocol:
    Model class for creating different ros operations.
"""

from dataclasses import dataclass
from typing import Any, Dict, Protocol

from ros2bridge.utils.data_parser import RosDataParser


@dataclass
class RosOperationsProtocol(Protocol):
    """
    Model class for creating different ros operations.

    Attributes:
        client: Dict[str, Any]
        data_parser: RosDataParser

    Method:
        handle_operation(self, data: Dict) -> None
            Run ws client based on request.
    """

    client: Dict[str, Any]
    data_parser: RosDataParser

    def handle_operation(self, data: Dict[str, Any]) -> None:
        """Run ws client based on request.

        Args:
            data (Dict): Request from ws client.
        """
        ...
