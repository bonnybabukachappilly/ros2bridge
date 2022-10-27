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

    def __init__(self) -> None:
        """PrePost."""
        with open('tests/utils/config.json', 'r') as _config:
            self.config = json.load(_config)

    def get_ws_url(self) -> str:
        """Generate WS url.

        Returns:
            str: WS url.
        """
        _url = self.config.get('ws_url')

        return f'{_url.get("domain")}://{_url.get("host")}:{_url.get("port")}'

    def get_constants(self, constants: str) -> str:
        """Test constants.

        Args:
            constants (str): Required constant key.

        Returns:
            str: Constant value.
        """
        return self.config.get('constants').get(constants)
