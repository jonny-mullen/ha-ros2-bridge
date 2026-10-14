# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ROS 2 Bridge integration.

Phase 0 ships the minimum that hassfest needs. The config flow, the MQTT hub, the
services and the Assist intents arrive in phase 3.
"""

from __future__ import annotations

from homeassistant.core import HomeAssistant
from homeassistant.helpers.typing import ConfigType

from .const import DOMAIN


async def async_setup(hass: HomeAssistant, config: ConfigType) -> bool:
    """Create the integration's shared container; it outlives config entry reloads."""
    hass.data.setdefault(DOMAIN, {})
    return True
