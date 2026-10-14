# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Windows-only stand-in for the Unix resource module (HA's runner imports it)."""

RLIMIT_NOFILE = 7
RLIM_INFINITY = -1


def getrlimit(which):
    """Report a generous fixed limit."""
    return (4096, 4096)


def setrlimit(which, limits):
    """No-op."""
    return
