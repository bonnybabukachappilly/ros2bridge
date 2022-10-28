"""ROS 2 BRIDGE PACKAGE.

This is for running the ros2bridge as a package.

Arguments:
    -p, --port:
        Set port for websocket, Default is 9020
        This is an optional argument if needed to override defaults.

    -n, --ngrok:
        This is an optional flag.
        If set, websocket is hosted internally rather than on local ip.
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
