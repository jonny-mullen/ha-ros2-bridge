# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The Python-to-ROS log handler against rclpy's per-call-site severity rule.

rclpy's ``RcutilsLogger`` remembers the severity of every call site (file,
line and function of the caller) and raises ``ValueError('Logger severity
cannot be changed between calls.')`` when the same site logs at another
severity. The first live run of the node hit exactly that: one line in the
handler forwarded every level, so the second distinct level lost its message
to ``handleError``. This stub enforces the same rule without rclpy, so the
test runs on any machine.
"""

from __future__ import annotations

import logging
import sys

import pytest

from ros2_ha_bridge.ros_adapters import RclpyLogHandler


class StrictStubRosLogger:
    """Records calls like rclpy would and applies rclpy's call-site severity rule."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []
        self._severity_by_site: dict[tuple[str, int], str] = {}

    def _log(self, severity: str, message: str) -> None:
        frame = sys._getframe(2)
        site = (frame.f_code.co_filename, frame.f_lineno)
        seen = self._severity_by_site.setdefault(site, severity)
        if seen != severity:
            raise ValueError('Logger severity cannot be changed between calls.')
        self.calls.append((severity, message))

    def debug(self, message: str) -> None:
        self._log('debug', message)

    def info(self, message: str) -> None:
        self._log('info', message)

    def warning(self, message: str) -> None:
        self._log('warning', message)

    def error(self, message: str) -> None:
        self._log('error', message)

    def fatal(self, message: str) -> None:
        self._log('fatal', message)


def test_every_level_reaches_ros_from_its_own_call_site(monkeypatch: pytest.MonkeyPatch) -> None:
    stub = StrictStubRosLogger()
    handler = RclpyLogHandler(stub)
    errors: list[logging.LogRecord] = []
    monkeypatch.setattr(handler, 'handleError', errors.append)
    logger = logging.getLogger('test_ros_log_handler.levels')
    logger.propagate = False
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    try:
        for _ in range(2):
            logger.debug('d')
            logger.info('i')
            logger.warning('w')
            logger.error('e')
            logger.critical('c')
    finally:
        logger.removeHandler(handler)
    assert errors == [], 'a severity change at a shared call site reached handleError'
    assert [severity for severity, _ in stub.calls] == [
        'debug',
        'info',
        'warning',
        'error',
        'fatal',
    ] * 2
