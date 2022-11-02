"""Bridge Node.

ROS node for ros2bridge.

Class:
    RosWSBridge(Node):
        ROS2 Node for creating bridge.
"""

from rclpy.node import Node

from ros2bridge.server.ws_server import WebSocketServer

from std_msgs.msg import String


class RosWSBridge(Node):
    """Ros Bridge.

    ROS2 Node for creating bridge.

    Attributes:
        client_number:
            Publisher for publishing number of connected clients.
        clients_list:
            Publisher for publishing list of connected clients.

    Methods:
        publish_client:
            Publish message when timer expires.

    """

    def __init__(self):
        """Ros Bridge."""
        super().__init__('ros2bridge')

        self.client_number = self.create_publisher(
            String,
            'ros2bridge/total_client',
            10
        )

        self.clients_list = self.create_publisher(
            String,
            'ros2bridge/clients_list',
            10
        )

        self.timer = self.create_timer(1, self.publish_client)

    def publish_client(self):
        """Publish message when timer expires."""
        msg_clients_list = String()
        msg_clients_no = String()

        _clients = list(WebSocketServer.connected_clients.keys())

        msg_clients_list.data = f'List of clients: {_clients}'
        msg_clients_no.data = f'Number of clients: {len(_clients)}'
        self.clients_list.publish(msg_clients_list)
        self.client_number.publish(msg_clients_no)
