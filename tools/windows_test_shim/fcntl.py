# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Windows-only stand-in for the Unix fcntl module.

homeassistant.runner imports fcntl at module level (used only for the config
directory lock when running HA itself). The test harness imports runner, so this
stub lets the suite import on Windows; nothing in the tests calls these.
"""

LOCK_EX = 2
LOCK_NB = 4
LOCK_SH = 1
LOCK_UN = 8


def flock(fd, operation):
    """No-op lock."""
    return


def fcntl(fd, cmd, arg=0):
    """No-op fcntl."""
    return 0
