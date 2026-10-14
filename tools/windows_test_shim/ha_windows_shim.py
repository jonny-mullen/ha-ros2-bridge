# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Local pytest plugin: let the Windows asyncio self-pipe exist under pytest-socket.

pytest-homeassistant-custom-component blocks every non-Unix socket for each test.
On Linux asyncio's self-pipe is an AF_UNIX socketpair, which is allowed; on Windows
socket.socketpair() falls back to two AF_INET sockets on 127.0.0.1, which the guard
refuses, so no event loop can be created. This plugin replaces socket.socketpair with
the same fallback built on the real socket class captured before the guard is
installed. 127.0.0.1 is already on the harness's allow-hosts list. Loaded with
`-p ha_windows_shim`; it is a developer convenience for Windows only; CI runs the suite natively on Ubuntu.
"""

from __future__ import annotations

import socket as _socket

_REAL_SOCKET = _socket.socket


def _socketpair(family=_socket.AF_INET, type=_socket.SOCK_STREAM, proto=0):
    if family == getattr(_socket, "AF_UNIX", object()):
        return _ORIGINAL_SOCKETPAIR(family, type, proto)
    host = "127.0.0.1" if family == _socket.AF_INET else "::1"
    lsock = _REAL_SOCKET(family, type, proto)
    try:
        lsock.bind((host, 0))
        lsock.listen()
        addr, port = lsock.getsockname()[:2]
        csock = _REAL_SOCKET(family, type, proto)
        try:
            csock.setblocking(False)
            try:
                csock.connect((addr, port))
            except (BlockingIOError, InterruptedError):
                pass
            csock.setblocking(True)
            fd, _ = lsock._accept()
            ssock = _REAL_SOCKET(family, type, proto, fileno=fd)
        except Exception:
            csock.close()
            raise
    finally:
        lsock.close()
    return (ssock, csock)


_ORIGINAL_SOCKETPAIR = _socket.socketpair


def pytest_configure(config) -> None:
    _socket.socketpair = _socketpair


# ---------------------------------------------------------------------------
# homeassistant.helpers.frame.get_integration_frame looks for "custom_components/"
# in frame filenames. Windows paths use backslashes, so on this machine every
# report_usage() from the integration raises MissingIntegrationFrame and is then
# treated as core code (RuntimeError) instead of a custom integration (a logged
# warning, as on Linux CI). This mirrors the upstream function with the path
# separators normalised. Developer convenience for Windows only.


def _install_frame_fix() -> None:
    import sys as _sys

    from homeassistant.helpers import frame as _frame

    def get_integration_frame(exclude_integrations=None):
        found_frame = None
        if not exclude_integrations:
            exclude_integrations = set()
        frame = _frame.get_current_frame()
        index = 0
        integration = ""
        path = ""
        while frame is not None:
            filename = frame.f_code.co_filename.replace("\\", "/")
            for path in ("custom_components/", "homeassistant/components/"):
                try:
                    index = filename.index(path)
                    start = index + len(path)
                    end = filename.index("/", start)
                    integration = filename[start:end]
                    if integration not in exclude_integrations:
                        found_frame = frame
                    break
                except ValueError:
                    continue
            if found_frame is not None:
                break
            frame = frame.f_back
        if found_frame is None:
            raise _frame.MissingIntegrationFrame
        found_module = None
        for module, module_obj in dict(_sys.modules).items():
            if not hasattr(module_obj, "__file__"):
                continue
            if module_obj.__file__ == found_frame.f_code.co_filename:
                found_module = module
                break
        return _frame.IntegrationFrame(
            custom_integration=path == "custom_components/",
            integration=integration,
            module=found_module,
            relative_filename=found_frame.f_code.co_filename.replace("\\", "/")[index:],
            frame=found_frame,
        )

    _frame.get_integration_frame = get_integration_frame


_ORIGINAL_CONFIGURE = pytest_configure


def pytest_configure(config) -> None:
    _ORIGINAL_CONFIGURE(config)
    _install_frame_fix()
