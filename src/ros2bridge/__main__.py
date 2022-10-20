"""
ROS2BRIDGE.

Python module for creating a bridge between ROS DDS and Websocket.
"""

from argparse import ArgumentParser

from ros2bridge.ros2bridge import bridge

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument(
        '-p', '--port', required=False, default='9020',
        help='Set port for websocket, if not provided 9020'
    )
    parser.add_argument(
        '-n', '--ngrok', action='store_true',
        help='If not provided, will host on ip else on internal for ngrok')

    args = parser.parse_args()
    bridge(args=args)
