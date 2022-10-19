"""Test utilities."""

import json


class PrePost:
    """
    Create Startup and teardown for test.

    Methods
    -------
    get_ws_url(self) -> str:
        Get WS Url
    """

    def get_ws_url(self) -> str:
        """Generate WS url.

        Returns:
            str: WS url.
        """
        with open('tests/utils/config.json', 'r') as _config:
            config = json.load(_config)

        _url = config.get('ws_url')

        return f'{_url.get("domain")}://{_url.get("host")}:{_url.get("port")}'
