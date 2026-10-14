#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Render the ROS 2 parameters file the Docker rig's entrypoint hands to the node.

``tools/entrypoint.sh`` calls this before ``ros2 launch`` or ``ros2 run``:

    render_params.py OUT

It starts from the base file named by ``ROS2_HA_BRIDGE_PARAMS_FILE`` (mounted
into the container; optional) and applies the environment overrides to the one
node entry that file names, or to ``/**`` (every node). Two kinds of variable
are read:

- the six named variables of the rig: ``ROS2_HA_BRIDGE_ROBOT_ID`` (robot.id),
  ``ROS2_HA_BRIDGE_ROBOT_NAME`` (robot.name), ``MQTT_HOST`` (mqtt.host),
  ``MQTT_PORT`` (mqtt.port), ``MQTT_USERNAME`` (mqtt.username) and
  ``TOPIC_PREFIX`` (topic_prefix);
- the generic form ``ROS2_HA_BRIDGE_PARAM_<NAME>`` for ANY node parameter in
  ``docs/configuration.md``, where ``<NAME>`` is the dotted parameter name in
  upper case with every dot written as a double underscore:
  ``ROS2_HA_BRIDGE_PARAM_WATCHDOG__MODE=inhibit_new`` sets ``watchdog.mode``,
  ``ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED=true`` sets ``mqtt.tls.enabled``.

A value is converted to the parameter's declared type (the table in
``ros2_ha_bridge.parameters``, plus the ``moveit.*`` scalars the MoveIt
executor declares): an integer parameter needs an integer, a double any number,
a bool a YAML boolean (``true`` or ``false``), a string array either a
comma-separated list (``reset,stop``) or a YAML flow list (``[reset, stop]``),
and a string parameter takes the text as it is. An unset or empty variable is
no override. A name that is not a parameter, a value of the wrong type, and the
two secrets (``mqtt.password`` and ``command.secret``, which the node reads from
``ROS2_HA_BRIDGE_MQTT_PASSWORD`` and ``ROS2_HA_BRIDGE_COMMAND_SECRET`` itself)
fail the render, which stops the container before the node starts.

The rendered file is written as nested maps under ``ros__parameters`` (rcl
flattens them to the dotted names); a parameter whose parent is itself a value
(``executor.call_warn_s`` under ``executor: fake``) is written as a dotted key,
which rcl reads the same way. Any earlier spelling of the same parameter in the
base file is removed first, so the override always wins; a declared parameter
beneath the overridden name (``executor.call_warn_s`` when ``executor`` is
overridden) is kept, because rcl reads it as a parameter of its own, while an
undeclared subtree (inline ``moveit.poses``) is the value and goes with it.

The module is importable (``render(environ)``) so the ROS test suite drives it
without a container; the generic form needs ``ros2_ha_bridge`` importable,
which the entrypoint guarantees by sourcing the workspace first.
"""

from __future__ import annotations

import math
import os
import sys
from collections.abc import Collection, Mapping, MutableMapping
from dataclasses import dataclass
from typing import Any

import yaml

__all__ = [
    "ENV_BASE_FILE",
    "GENERIC_PREFIX",
    "NAMED_VARIABLES",
    "SECRET_PARAMETERS",
    "Override",
    "RenderError",
    "RenderResult",
    "main",
    "parameter_kinds",
    "parameter_name",
    "render",
    "set_parameter",
    "variable_name",
]

#: The base parameters file (mounted into the container); empty means start from nothing.
ENV_BASE_FILE = "ROS2_HA_BRIDGE_PARAMS_FILE"
#: Prefix of the generic override variables.
GENERIC_PREFIX = "ROS2_HA_BRIDGE_PARAM_"
#: The named variables kept for compatibility: variable to (parameter, kind).
NAMED_VARIABLES: Mapping[str, tuple[str, str]] = {
    "ROS2_HA_BRIDGE_ROBOT_ID": ("robot.id", "string"),
    "ROS2_HA_BRIDGE_ROBOT_NAME": ("robot.name", "string"),
    "MQTT_HOST": ("mqtt.host", "string"),
    "MQTT_PORT": ("mqtt.port", "integer"),
    "MQTT_USERNAME": ("mqtt.username", "string"),
    "TOPIC_PREFIX": ("topic_prefix", "string"),
}
#: Parameters the rig refuses to render: the node reads them from its own environment variables.
SECRET_PARAMETERS: Mapping[str, str] = {
    "mqtt.password": "ROS2_HA_BRIDGE_MQTT_PASSWORD",
    "command.secret": "ROS2_HA_BRIDGE_COMMAND_SECRET",
}
#: The node entry overrides go to when the base file names none or several.
EVERY_NODE = "/**"
ROS_PARAMETERS = "ros__parameters"


class RenderError(Exception):
    """A variable the rig cannot render; the message names it."""


@dataclass(frozen=True)
class Override:
    """One environment override: the variable it came from, the parameter and the typed value."""

    variable: str
    name: str
    value: Any


@dataclass(frozen=True)
class RenderResult:
    """The rendered document and the overrides applied to it, in application order."""

    document: dict[str, Any]
    overrides: tuple[Override, ...]


# --------------------------------------------------------------------------- names and kinds


def variable_name(parameter: str) -> str:
    """The generic variable for a dotted parameter name (``mqtt.tls.enabled`` to ``..._MQTT__TLS__ENABLED``)."""
    return GENERIC_PREFIX + parameter.upper().replace(".", "__")


def parameter_name(variable: str) -> str:
    """The dotted parameter name a generic variable addresses."""
    if not variable.startswith(GENERIC_PREFIX):
        raise ValueError(f"{variable} does not start with {GENERIC_PREFIX}")
    return variable[len(GENERIC_PREFIX) :].lower().replace("__", ".")


def _kind_of_default(default: object) -> str:
    if isinstance(default, bool):
        return "bool"
    if isinstance(default, int):
        return "integer"
    if isinstance(default, float):
        return "double"
    if isinstance(default, (list, tuple)):
        return "string_array"
    return "string"


def parameter_kinds() -> dict[str, str]:
    """Parameter name to kind for every parameter the node or the MoveIt executor declares.

    Raises :class:`RenderError` when ``ros2_ha_bridge`` is not importable
    (the workspace is not sourced), since a generic override cannot be typed
    without the table.
    """
    try:
        from ros2_ha_bridge.parameters import PARAMETERS
    except ImportError as error:
        raise RenderError(
            "a ROS2_HA_BRIDGE_PARAM_* variable needs the ros2_ha_bridge package importable "
            f"(source the workspace first): {error}"
        ) from error
    kinds: dict[str, str] = {spec.name: spec.kind for spec in PARAMETERS}
    try:
        from ros2_ha_bridge.executors.moveit import SCALAR_PARAMETERS
    except ImportError:
        return kinds
    for name, default, _ in SCALAR_PARAMETERS:
        kinds.setdefault(name, _kind_of_default(default))
    return kinds


def declared_names() -> frozenset[str]:
    """Every declared parameter name, or an empty set when ``ros2_ha_bridge`` is not importable.

    The named variables alone need no table (none of their targets has a
    declared parameter beneath it), so an unsourced workspace is not an error here.
    """
    try:
        return frozenset(parameter_kinds())
    except RenderError:
        return frozenset()


# --------------------------------------------------------------------------- values


def convert(variable: str, name: str, kind: str, raw: str) -> Any:
    """``raw`` as the Python value of a ``kind`` parameter, or :class:`RenderError`."""
    if kind == "string":
        return raw
    if kind == "string_array":
        text = raw.strip()
        if text.startswith("["):
            loaded = _yaml_scalar(variable, raw)
            if not isinstance(loaded, list) or not all(isinstance(i, str) for i in loaded):
                raise RenderError(
                    f"{variable}={raw!r}: {name} needs a list of strings, for example "
                    "[reset, stop] or reset,stop"
                )
            return list(loaded)
        return [item.strip() for item in text.split(",") if item.strip()]
    loaded = _yaml_scalar(variable, raw)
    if kind == "bool":
        if not isinstance(loaded, bool):
            raise RenderError(f"{variable}={raw!r}: {name} needs true or false")
        return loaded
    if kind == "integer":
        if isinstance(loaded, bool) or not isinstance(loaded, int):
            raise RenderError(f"{variable}={raw!r}: {name} needs an integer")
        return loaded
    if kind == "double":
        if isinstance(loaded, bool) or not isinstance(loaded, (int, float)):
            raise RenderError(f"{variable}={raw!r}: {name} needs a number")
        number = float(loaded)
        if not math.isfinite(number):
            raise RenderError(f"{variable}={raw!r}: {name} needs a finite number")
        return number
    raise RenderError(f"{variable}: {name} has an unknown kind {kind!r}")


def _yaml_scalar(variable: str, raw: str) -> Any:
    try:
        return yaml.safe_load(raw)
    except yaml.YAMLError as error:
        raise RenderError(f"{variable}={raw!r} is not valid YAML: {error}") from error


# --------------------------------------------------------------------------- the document


def load_base(path: str) -> dict[str, Any]:
    """The base file as a mapping, or an empty document when ``path`` is empty."""
    if not path:
        return {}
    try:
        with open(path, encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}
    except OSError as error:
        raise RenderError(f"cannot read {ENV_BASE_FILE} {path!r}: {error}") from error
    except yaml.YAMLError as error:
        raise RenderError(f"{ENV_BASE_FILE} {path!r} is not valid YAML: {error}") from error
    if not isinstance(loaded, dict):
        raise RenderError(f"{path!r} must be a mapping (a ROS 2 parameters file)")
    return loaded


def params_of(document: dict[str, Any]) -> dict[str, Any]:
    """The ``ros__parameters`` mapping the overrides go to (created when absent)."""
    entries = [k for k, v in document.items() if isinstance(v, dict) and ROS_PARAMETERS in v]
    key = entries[0] if len(entries) == 1 else EVERY_NODE
    node = document.setdefault(key, {})
    if not isinstance(node, dict):
        raise RenderError(f"node entry {key!r} in the base file is not a mapping")
    params = node.setdefault(ROS_PARAMETERS, {})
    if not isinstance(params, dict):
        raise RenderError(f"{ROS_PARAMETERS} of {key!r} in the base file is not a mapping")
    return params


def _leads_to_declared(full: str, declared: Collection[str]) -> bool:
    """True when ``full`` is a declared parameter or a prefix of one."""
    return any(name == full or name.startswith(full + ".") for name in declared)


def remove_parameter(
    params: MutableMapping[str, Any],
    name: str,
    prefix: str = "",
    declared: Collection[str] = (),
) -> None:
    """Delete every spelling of ``name`` (nested, dotted or mixed), keeping the declared
    parameters beneath it.

    rcl flattens a nested mapping to dotted names, so ``executor: {call_warn_s: 0.2}``
    and ``executor.call_warn_s: 0.2`` are one parameter, and a different one from
    ``executor``. A key beneath ``name`` survives when it is, or leads to, a name in
    ``declared``; one that sat inside the removed mapping is re-homed as a dotted key
    at this level. Anything else beneath ``name`` (inline ``moveit.poses``, whose
    entries no table declares) is the parameter's value and goes with it.
    """
    for key in list(params):
        full = f"{prefix}{key}"
        value = params[key]
        if full == name:
            del params[key]
            if isinstance(value, dict):
                for child, child_value in value.items():
                    if _leads_to_declared(f"{full}.{child}", declared):
                        params.setdefault(f"{key}.{child}", child_value)
        elif full.startswith(name + "."):
            if not _leads_to_declared(full, declared):
                del params[key]
        elif isinstance(value, dict) and (name + ".").startswith(full + "."):
            remove_parameter(value, name, full + ".", declared)


def set_parameter(
    params: MutableMapping[str, Any], name: str, value: Any, declared: Collection[str] = ()
) -> None:
    """Write ``name`` nested where the path is free, as a dotted key where a value sits on it.

    ``declared`` is the set of parameter names whose spellings beneath ``name``
    must survive (:func:`remove_parameter`).
    """
    remove_parameter(params, name, declared=declared)
    keys = name.split(".")
    cursor: MutableMapping[str, Any] = params
    for index, key in enumerate(keys[:-1]):
        nxt = cursor.get(key)
        if nxt is None:
            nxt = {}
            cursor[key] = nxt
        elif not isinstance(nxt, dict):
            # A scalar parameter sits on the path (executor: fake under executor.call_warn_s):
            # rcl reads the dotted spelling at this level as the same parameter.
            cursor[".".join(keys[index:])] = value
            return
        cursor = nxt
    cursor[keys[-1]] = value


# --------------------------------------------------------------------------- rendering


def overrides_from(environ: Mapping[str, str]) -> list[Override]:
    """Every override the environment asks for: the named variables first, then the generic ones."""
    found: list[Override] = []
    for variable, (name, kind) in NAMED_VARIABLES.items():
        raw = environ.get(variable, "")
        if raw == "":
            continue
        found.append(Override(variable, name, convert(variable, name, kind, raw)))
    generic = sorted(v for v in environ if v.startswith(GENERIC_PREFIX) and environ[v] != "")
    kinds = parameter_kinds() if generic else {}
    for variable in generic:
        name = parameter_name(variable)
        if name in SECRET_PARAMETERS:
            raise RenderError(
                f"{variable} is refused: the rig never writes {name} into a file; the node "
                f"reads it from {SECRET_PARAMETERS[name]}"
            )
        declared = kinds.get(name)
        if declared is None:
            raise RenderError(
                f"{variable} names no parameter: {name!r} is not declared by the node "
                "(docs/configuration.md lists them; dots are written as double underscores)"
            )
        value = convert(variable, name, declared, environ[variable])
        found.append(Override(variable, name, value))
    return found


def render(environ: Mapping[str, str] | None = None) -> RenderResult:
    """The document for ``environ`` (default ``os.environ``): base file plus overrides."""
    env = os.environ if environ is None else environ
    document = load_base(env.get(ENV_BASE_FILE, ""))
    overrides = overrides_from(env)
    if overrides:
        params = params_of(document)
        declared = declared_names()
        for override in overrides:
            set_parameter(params, override.name, override.value, declared)
    elif not document:
        document = {EVERY_NODE: {ROS_PARAMETERS: {}}}
    return RenderResult(document, tuple(overrides))


def write(result: RenderResult, out: str) -> None:
    """Write the rendered document to ``out`` (its directory is created)."""
    directory = os.path.dirname(out)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(out, "w", encoding="utf-8") as handle:
        yaml.safe_dump(result.document, handle, default_flow_style=False, sort_keys=False)


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print("usage: render_params.py OUT", file=sys.stderr)
        return 2
    out = args[0]
    try:
        result = render()
        write(result, out)
    except RenderError as error:
        print(f"entrypoint: {error}", file=sys.stderr)
        return 1
    if result.overrides:
        shown = ", ".join(f"{o.name}={o.value!r}" for o in result.overrides)
        print(f"entrypoint: rendered {out} with overrides {shown}", file=sys.stderr)
    else:
        print(f"entrypoint: rendered {out} with no environment overrides", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
