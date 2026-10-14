# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Executor plugins: the interface in :mod:`ros2_ha_bridge.executors.base` and the shipped ones.

Only the interface is imported here, so importing this package never pulls in a
particular plugin's dependencies. Plugins are found by :func:`load_executor`
through the ``ros2_ha_bridge.executors`` entry-point group or by dotted path.
"""

from ros2_ha_bridge.executors.base import (
    ENTRY_POINT_GROUP,
    BridgeContext,
    ExecutorLoadError,
    TaskExecutor,
    TaskHandle,
    TaskRejected,
    TaskRequest,
    TaskResult,
    TaskSpec,
    load_executor,
)

__all__ = [
    'ENTRY_POINT_GROUP',
    'BridgeContext',
    'ExecutorLoadError',
    'TaskExecutor',
    'TaskHandle',
    'TaskRejected',
    'TaskRequest',
    'TaskResult',
    'TaskSpec',
    'load_executor',
]
