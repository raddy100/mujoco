# MuJoCo Copilot Instructions

MuJoCo is a physics engine written in C, with C++ for tests/plugins/bindings, Python bindings, and a Unity (C#) integration. Current version: 3.3.8.

## Build

Requires CMake >= 3.16.

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DMUJOCO_BUILD_TESTS=ON \
         -DMUJOCO_BUILD_EXAMPLES=ON \
         -DMUJOCO_BUILD_SIMULATE=ON
cmake --build . --config Release
```

Ninja is the preferred generator. On Windows, Visual Studio generators are also supported.

Key CMake options:
- `MUJOCO_BUILD_TESTS` – build C++ test suite
- `MUJOCO_BUILD_SIMULATE` – build the interactive simulator
- `MUJOCO_WITH_USD` – enable OpenUSD support (`src/experimental/usd/`)
- `mjUSEPLATFORMSIMD` – enable AVX intrinsics

## Tests

```bash
# All tests
cd build && ctest -C Release --output-on-failure

# Single test by name pattern
ctest -R <test_name_pattern> -C Release --output-on-failure

# Run a test binary directly (faster iteration)
./bin/Release/<test_binary>  # e.g. engine_core_smooth_test
```

- C++ tests use **GoogleTest** (`test/engine/`, `test/user/`, `test/xml/`, `test/plugin/`)
- Python tests use **absltest** (`python/tests/`)
- C# tests use **NUnit** (Unity integration)
- Performance benchmarks are in `test/benchmark/`

All PRs must pass existing tests. Most PRs require new tests.

## Formatting & Linting

**C/C++** — `clang-format` with `.clang-format` at the repo root (Microsoft base style, 4-space indent, 120-char line limit):
```bash
clang-format -i <file>
```

**Python** — `pyink` (Google style) + `isort`:
```bash
pyink <file> && isort <file>
```

All compiler warnings must be resolved before merging.

## Architecture

```
include/mujoco/      # Public C API headers (12 headers, one per subsystem)
src/
  engine/            # Core physics: collision, dynamics, solver, visualization (~36 .c files)
  user/              # Model compilation (mjSpec → mjModel)
  xml/               # XML/URDF/MJCF parser
  render/            # OpenGL rendering
  ui/                # GUI framework
  thread/            # Threading utilities
  experimental/usd/  # USD/Houdini integration
plugin/              # Built-in plugins: actuator, sensor, elasticity, sdf
python/mujoco/       # Python bindings (ctypes + pybind11 wrappers)
mjx/                 # JAX-based MuJoCo (mjx subpackage)
test/                # C++ test suite (mirrors src/ layout)
simulate/            # Interactive viewer application
sample/              # Standalone example programs
```

The core library (`mujoco` shared lib) is pure C. C++ is used only for tests, plugins, Python bindings, and the simulator UI. The two central data structures are `mjModel` (static model, read-only at runtime) and `mjData` (runtime simulation state, one per thread for parallelism).

## Key Conventions

### C API Naming
- Public functions: `mj_functionName()` / `mju_utilityName()`
- Types/structs: `mjTypeName` (e.g., `mjModel`, `mjData`, `mjSpec`)
- Enums: `mjtEnumName` (e.g., `mjtJoint`, `mjtDisableBit`)
- Plugin types: `mjpPluginType` (e.g., `mjpPlugin`)
- Callbacks: `mjfCallbackType` (e.g., `mjfGeneric`)
- Constants: `mjNAME` (e.g., `mjNTIMER`, `mjNDISABLE`)

### C Code Style (see STYLEGUIDE.md for full details)
- 2-space indentation (not 4, despite `.clang-format` base style — follow existing code)
- 100-char line length
- Mandatory braces on all `if`/`for`/`while` blocks
- Variable declarations at narrowest scope (C99 style preferred; legacy C89-style is being migrated out)
- Comments: single-line, **uncapitalized**, **no trailing period**, empty line before (except at block start)
- Public API docstrings: **capitalized**, **with trailing period**, on the declaration in the header

### Headers
- Public headers live in `include/mujoco/` with `#ifndef MUJOCO_INCLUDE_MJXXX_H_` guards
- All files begin with the Apache 2.0 license header
- C headers use `extern "C"` guards for C++ compatibility

### Plugin System
Plugins extend MuJoCo via capability flags: `mjPLUGIN_ACTUATOR`, `mjPLUGIN_SENSOR`, `mjPLUGIN_PASSIVE`, `mjPLUGIN_SDF`. See `include/mujoco/mjplugin.h` and `plugin/` for examples.

Global callbacks (`mjcb_control`, `mjcb_passive`, `mjcb_sensor`, etc.) and user-overridable handlers (`mju_user_error`, `mju_user_warning`, `mju_user_malloc`, `mju_user_free`) are the primary extension points at runtime.

### Python Bindings
`python/mujoco/` wraps the C API with Pythonic classes. Each subsystem has paired `.cc` / `.py` files (e.g., `structs.cc` + `structs.py`). The Python package version matches MuJoCo (with optional `.postN` suffix for bindings-only releases). `mjx/` is a separate JAX-based subpackage that shares the `mujoco` namespace.
