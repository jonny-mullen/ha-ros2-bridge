# Windows test shim for the Home Assistant suite

The Home Assistant test harness (`pytest-homeassistant-custom-component`) only runs natively on
Linux: it imports the POSIX-only `fcntl` and `resource` modules, its socket guard refuses the
AF_INET socket pair that asyncio needs on Windows, and `homeassistant.helpers.frame` looks for
`custom_components/` with forward slashes. CI runs the suite natively on Ubuntu and needs none of
this. On a Windows development PC, run the suite through this shim instead:

```bash
PYTHONPATH=tools/windows_test_shim .venv-ha/Scripts/python.exe -m pytest tests -q -p ha_windows_shim
```

The shim stubs `fcntl` and `resource` (nothing in the tests calls them), builds the asyncio
self-pipe from the real socket class, and normalises path separators in the integration-frame
lookup so `report_usage` behaves as it does on Linux. It changes no test semantics and is never
loaded unless named with `-p`.
