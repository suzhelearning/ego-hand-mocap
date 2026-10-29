import subprocess
import sys

import pytest


@pytest.mark.parametrize(
    ("module", "expected"),
    [
        ("natnet_zenoh.publisher", "--motive-server-ip"),
        ("natnet_zenoh.subscriber", "--listen-endpoint"),
    ],
)
def test_modules_expose_cli_help(module, expected):
    result = subprocess.run(
        [sys.executable, "-m", module, "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert expected in result.stdout
