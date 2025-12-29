# GitHub Copilot / AI Agent Instructions for ArduPilot

Purpose: give AI coding agents fast, actionable context to be productive in this repository.

- **Big picture**: ArduPilot is organized as vehicle top-level trees (e.g. `ArduCopter`, `ArduPlane`, `Rover`) plus shared code in `libraries/` and third-party or protocol code in `modules/`. Hardware Abstraction Layer implementations live in `libraries/AP_HAL*` (e.g. `AP_HAL_SITL`, `AP_HAL_Linux`, `AP_HAL_ESP32`).

- **Key directories**:
  - `libraries/` : shared subsystems (`AP_Param`, `AP_Logger`, `AP_Math`, sensors, drivers).
  - `modules/` : external libraries and generators (e.g. `mavlink/pymavlink`, `gsoap`).
  - `ArduCopter/`, `ArduPlane/`, `Rover/`, `ArduSub/` : vehicle-specific top-level code.
  - `Tools/` and `build/` : build helpers and generated outputs.

- **Build system / workflows**:
  - Waf is the single canonical build tool. Read build steps in [BUILD.md](BUILD.md) and the high-level project intro in [README.md](README.md).
  - Common commands (run from repo root):
    - Configure SITL: `./waf configure --board sitl`
    - Build vehicle: `./waf rover` or `./waf copter` or `./waf plane`
    - Build a single target: `./waf --targets bin/arducopter`
    - List boards: `./waf list_boards`
    - Run unit tests: `./waf --targets tests/test_math` (see `tests/`)
  - Do NOT run `waf` with `sudo` (permissions and environment breakage).

- **Auto-generated code & generators**:
  - Many bindings and protocol code are generated; look for comments like "auto generated bindings" in `libraries/AP_Scripting/generator` and `modules/mavlink/pymavlink/generator`.
  - Generated files often live under `build/` for a given board.

- **Project-specific patterns** (explicit, discoverable):
  - `AP_` prefix for libraries and subsystems (e.g. `AP_Param` for parameters, `AP_Logger` for logging).
  - HAL separation: platform abstraction is in `libraries/AP_HAL*`; new hardware targets usually add an `AP_HAL` implementation.
  - Parameters: search for `AP_Param::` and `AP_GROUPINFO` macros to understand param declaration patterns.

- **Where to look for examples**:
  - Build/config: `wscript`, `waf`, and `BUILD.md`.
  - Vehicle entrypoints: `ArduCopter/ArduCopter.cpp`, `Rover/` main files.
  - HALs: `libraries/AP_HAL_SITL/` and `libraries/AP_HAL_Linux/` show how the HAL is implemented for simulator and Linux.

- **Testing & CI**:
  - Unit and SITL tests are under `tests/` and exercised by GitHub Actions; see `.github/workflows/*.yml` for CI flows.
  - Use `./waf --targets tests/<testname>` for local runs.

- **If you edit generated code**: prefer editing the generator or source template (search for the generator path referenced in the generated file's header) rather than hand-editing the file in `build/`.

- **Quick pointers for common tasks**:
  - Build SITL + run: `./waf configure --board sitl && ./waf rover`
  - Reconfigure for a board: `./waf configure --board <board-name>` then `./waf`.
  - Find where a parameter is declared: `grep -R "AP_GROUPINFO\|AP_Param" libraries/ | head`.

If any of these sections should be expanded with examples, local scripts, or extra file links, tell me which area to deepen and I'll iterate.
