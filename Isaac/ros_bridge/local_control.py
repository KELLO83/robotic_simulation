"""Small same-host control channel between ROS 2 Humble and Isaac Sim.

Isaac Sim 6 embeds Python 3.12, while this repository deliberately runs ROS 2
Humble with the system Python 3.10.  Keeping rclpy in the external adapter
avoids loading the Humble Python extension into Isaac's interpreter.
"""

from __future__ import annotations

import json
from pathlib import Path
import socket
from typing import Any


CONTROL_SOCKET_PATH = Path("/tmp/turtlebot3_isaac_control.sock")


class LocalControlServer:
    """Receive non-blocking JSON commands from the Humble adapter."""

    def __init__(self, socket_path: Path = CONTROL_SOCKET_PATH) -> None:
        """Bind the local datagram socket used for simulation controls."""
        self._socket_path = socket_path
        self._socket_path.unlink(missing_ok=True)
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.bind(str(self._socket_path))
        self._socket.setblocking(False)
        self._socket_path.chmod(0o666)

    def poll(self) -> list[dict[str, Any]]:
        """Return every currently queued and structurally valid command."""
        commands: list[dict[str, Any]] = []
        while True:
            try:
                payload = self._socket.recv(4096)
            except BlockingIOError:
                break

            try:
                command = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(command, dict) and isinstance(command.get("command"), str):
                commands.append(command)
        return commands

    def close(self) -> None:
        """Close the socket and remove only this adapter's known endpoint."""
        self._socket.close()
        self._socket_path.unlink(missing_ok=True)
