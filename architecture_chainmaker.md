# ChainMaker V2 — Architecture & Implementation Guide

This document provides everything an AI agent needs to implement ChainMaker V2: a multi-chain
3D block-placement GUI built on MuJoCo. It specifies exact data structures, API calls, file
layouts, and rendering pipelines so that code can be generated with minimal ambiguity.

**Target audience:** AI code-generation agents.
**Companion document:** `PRD_chainmaker.md` (requirements & rationale).
**Reference implementation:** `progui/` (V1 prototype — single-chain, physics-during-build).

---

## Table of Contents

1. [Scope & Phasing](#1-scope--phasing)
2. [Build System Setup](#2-build-system-setup)
3. [Directory & File Layout](#3-directory--file-layout)
4. [Data Structures](#4-data-structures)
5. [Application Lifecycle](#5-application-lifecycle)
6. [Rendering Pipeline — Build Stage](#6-rendering-pipeline--build-stage)
7. [UI Construction](#7-ui-construction)
8. [Keyboard & Mouse Handling](#8-keyboard--mouse-handling)
9. [Chain Operations](#9-chain-operations)
10. [Save / Load (JSON)](#10-save--load-json)
11. [Simulation Compilation Pipeline](#11-simulation-compilation-pipeline)
12. [Collision Filtering](#12-collision-filtering)
13. [Profiling & Performance Monitoring](#13-profiling--performance-monitoring)
14. [Constants & Configuration](#14-constants--configuration)
15. [Reference Patterns from progui/](#15-reference-patterns-from-progui)
16. [Implementation Checklist](#16-implementation-checklist)
17. [IPC Test Server](#17-ipc-test-server)
18. [Video Recording](#18-video-recording)
19. [Scope Status](#19-scope-status)
20. [Known Requirements & Invariants](#20-known-requirements--invariants)

---

## 1. Scope & Phasing

**Phase 1 (complete ✅):** Build Stage GUI — multi-chain block placement,
grid rendering with `mjv_initGeom()`, save/load JSON. No physics compilation.

**Phase 2 (complete ✅):** Simulation Stage — compile `ChainWorld` to `mjSpec`, run physics,
profiling overlay, video recording, weld constraints for loop closure, return to build mode.

**Phases 3–7:** Actuators, sensors, multi-robot, automation, manufacturing export. See PRD §12.

### 1.1 What "Done" Looks Like for Phase 1

- A standalone GLFW window with a 3D viewport and right-side UI panel
- User can place cubes along 6 directions (±X, ±Y, ±Z) on an integer grid
- User can change direction (turn), creating turn blocks
- User can create multiple chains branching from existing junction blocks
- User can switch between chains, delete blocks from the active chain
- Ghost preview shows next placement position
- Save/load the entire world to/from JSON
- No MuJoCo physics model is compiled or simulated during building
- Blocks render as colored cubes via `mjv_initGeom()` directly into `mjvScene`

---

## 2. Build System Setup

### 2.1 Root CMakeLists.txt Integration

The root `CMakeLists.txt` already includes `progui/` via a plain variable gate:

```cmake
# Line 228-229 in root CMakeLists.txt
if(PROGUI)
  add_subdirectory(progui)
endif()
```

Add ChainMaker using the same pattern. Insert immediately after the PROGUI block:

```cmake
if(CHAINMAKER)
  add_subdirectory(chainmaker)
endif()
```

### 2.2 chainmaker/CMakeLists.txt

Create `chainmaker/CMakeLists.txt` modeled on `progui/CMakeLists.txt`:

```cmake
project(
  chainmaker
  VERSION 0.1.0
  DESCRIPTION "ChainMaker - Multi-chain bead-on-string robot designer"
)

enable_language(C)
enable_language(CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SAMPLE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
  if(WIN32)
    set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -Wl,/DYNAMICBASE)
  else()
    set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${MUJOCO_SAMPLE_COMPILE_OPTIONS}" -fPIE)
    if(APPLE)
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -Wl,-pie)
    else()
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -pie)
    endif()
  endif()
endif()

# Fetch nlohmann/json for save/load
include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

add_executable(ChainMaker
  main.cc
  chainmaker.h
  chainmaker.cpp
  chainmaker_render.h
  chainmaker_render.cpp
  chainmaker_ui.h
  chainmaker_ui.cpp
  chainmaker_io.h
  chainmaker_io.cpp
  chainmaker_compile.h
  chainmaker_compile.cpp
  chainmaker_sim.h
  chainmaker_sim.cpp
)

target_compile_options(ChainMaker PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})

find_package(Threads REQUIRED)
target_link_libraries(
  ChainMaker
  mujoco::mujoco
  glfw
  Threads::Threads
  nlohmann_json::nlohmann_json
)

target_link_options(ChainMaker PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ChainMaker)
```

### 2.3 Build Commands

```bash
# Configure (from repo root)
cmake -B build -DCHAINMAKER=ON -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --config Release --target ChainMaker

# Run
./build/chainmaker/ChainMaker        # Linux/macOS
build\chainmaker\Release\ChainMaker   # Windows
```

### 2.4 Dependencies

| Dependency | Source | Purpose |
|------------|--------|---------|
| `mujoco::mujoco` | Parent project | Rendering, visualization, physics (Phase 2) |
| `glfw` | FetchContent (parent) | Window management, input handling |
| `Threads::Threads` | System | Thread support |
| `nlohmann_json::nlohmann_json` | FetchContent | JSON save/load |

### 2.5 C++ Standard

The project uses **C++20** (set in `cmake/MujocoOptions.cmake`). All ChainMaker code
should use C++20 features freely (structured bindings, `std::format` where available,
concepts, etc.).

### 2.6 Key MuJoCo Headers

```cpp
#include <mujoco/mujoco.h>       // Core API: mj_step, mj_forward, mj_compile, etc.
#include <mujoco/mjspec.h>       // Spec API: mjs_addBody, mjs_addGeom, mjs_addJoint, etc.
#include <mujoco/mjvisualize.h>  // Visualization: mjvScene, mjvGeom, mjv_initGeom, etc.
#include <mujoco/mjrender.h>     // Rendering: mjrContext, mjr_render, mjr_overlay
#include <mujoco/mjui.h>         // UI widgets: mjUI, mjuiDef, mjui_add, mjui_render
#include <GLFW/glfw3.h>          // Window, input callbacks
```

---

## 3. Directory & File Layout

```
chainmaker/
├── CMakeLists.txt              # Build configuration (§2.2)
├── main.cc                     # Entry point: GLFW window, render loop (§5)
├── chainmaker.h                # Core data structures: GridCell, Chain, ChainWorld (§4)
├── chainmaker.cpp              # Grid logic, chain operations, validation (§9)
├── chainmaker_render.h         # Rendering function declarations (§6)
├── chainmaker_render.cpp       # mjvScene population, floor, ghost, direction arrow (§6)
├── chainmaker_ui.h             # UI panel definitions, callback declarations (§7)
├── chainmaker_ui.cpp           # UI construction, button handlers, keyboard, mouse (§7, §8)
├── chainmaker_io.h             # Save/load function declarations (§10)
├── chainmaker_io.cpp           # JSON serialization/deserialization (§10)
├── chainmaker_compile.h        # Grid → mjSpec compiler declarations (§11)
├── chainmaker_compile.cpp      # Compilation logic, collision filtering (§11, §12)
├── chainmaker_sim.h            # Simulation stage declarations (§13)
└── chainmaker_sim.cpp          # Physics stepping, profiling overlay (§13)
```

### 3.1 File Responsibilities

| File | Phase | Responsibility |
|------|-------|----------------|
| `main.cc` | 1 | GLFW init, render loop, cleanup. Owns `GLFWwindow*`, `mjvScene`, `mjrContext`. |
| `chainmaker.h/cpp` | 1 | `ChainWorld` data structure, all chain mutation functions (place, delete, turn, new chain, validate). Pure logic — no rendering or IO. |
| `chainmaker_render.h/cpp` | 1 | Populate `mjvScene` from `ChainWorld` each frame. Floor, blocks, ghost, head marker, direction arrow. |
| `chainmaker_ui.h/cpp` | 1 | Build `mjUI` panel, handle button clicks. Keyboard and mouse GLFW callbacks. |
| `chainmaker_io.h/cpp` | 1 | Serialize `ChainWorld` to JSON, deserialize JSON to `ChainWorld`. Uses nlohmann/json. |
| `chainmaker_compile.h/cpp` | 2 | Convert `ChainWorld` → `mjSpec` with bodies, joints, geoms, collision filters. |
| `chainmaker_sim.h/cpp` | 2 | Run physics loop, profiling overlay, video recording, return-to-build. |

---

## 4. Data Structures

All data structures live in `chainmaker.h`. This is the most critical file — it defines
the entire data model that every other file operates on.

### 4.1 SpawnFace Enum

```cpp
// Direction of block placement (which face of the current head block to extend from)
enum SpawnFace : int {
    FACE_POS_X = 0,  // +X direction
    FACE_NEG_X = 1,  // -X direction
    FACE_POS_Y = 2,  // +Y direction
    FACE_NEG_Y = 3,  // -Y direction
    FACE_POS_Z = 4,  // +Z direction
    FACE_NEG_Z = 5,  // -Z direction
};

// Total number of faces (used for iteration/bounds checking)
constexpr int kNumFaces = 6;
```

### 4.2 IVec3 — Integer 3D Coordinate

We avoid depending on GLM. Use a simple struct with hash support for `unordered_map`:

```cpp
struct IVec3 {
    int x = 0, y = 0, z = 0;

    bool operator==(const IVec3& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
    bool operator!=(const IVec3& o) const { return !(*this == o); }

    IVec3 operator+(const IVec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    IVec3 operator-(const IVec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
};

// Hash for unordered_map
struct IVec3Hash {
    std::size_t operator()(const IVec3& v) const {
        // Combine with prime multipliers
        std::size_t h = std::hash<int>{}(v.x);
        h ^= std::hash<int>{}(v.y) * 2654435761u;
        h ^= std::hash<int>{}(v.z) * 40503u;
        return h;
    }
};
```

### 4.3 Direction Helpers

```cpp
// Convert SpawnFace to axis index (0=X, 1=Y, 2=Z) and sign (+1 or -1)
inline void FaceToAxisSign(SpawnFace face, int& axis, int& sign) {
    axis = static_cast<int>(face) / 2;  // 0,1 → 0; 2,3 → 1; 4,5 → 2
    sign = (static_cast<int>(face) % 2 == 0) ? +1 : -1;
}

// Convert axis index and sign back to SpawnFace
inline SpawnFace AxisSignToFace(int axis, int sign) {
    return static_cast<SpawnFace>(axis * 2 + (sign < 0 ? 1 : 0));
}

// Get the grid offset vector for a SpawnFace (unit step in that direction)
inline IVec3 FaceToOffset(SpawnFace face) {
    int axis, sign;
    FaceToAxisSign(face, axis, sign);
    IVec3 off{0, 0, 0};
    (&off.x)[axis] = sign;  // sets the correct component
    return off;
}

// Get the opposite face (reverse direction)
inline SpawnFace OppositeFace(SpawnFace face) {
    int f = static_cast<int>(face);
    return static_cast<SpawnFace>(f ^ 1);  // flip LSB: 0↔1, 2↔3, 4↔5
}
```

### 4.4 GridCell

```cpp
struct GridCell {
    IVec3 pos;                  // integer grid coordinates

    // Axis occupancy: which chain ID passes through this cell on each axis
    // -1 = unoccupied on that axis
    int chain_on_axis[3] = {-1, -1, -1};  // [X, Y, Z]

    // Turn block info
    bool is_turn = false;       // true if this is a turn block
    int turn_entry_axis = -1;   // axis the chain enters on (0=X, 1=Y, 2=Z)
    int turn_exit_axis = -1;    // axis the chain exits on

    // How many chains reference this cell
    int ChainCount() const {
        int n = 0;
        for (int i = 0; i < 3; i++) {
            if (chain_on_axis[i] >= 0) n++;
        }
        return n;
    }

    // Is this cell a junction (shared by 2+ chains)?
    bool IsJunction() const { return ChainCount() >= 2; }

    // Is the given axis available for a new chain?
    bool IsAxisFree(int axis) const {
        return axis >= 0 && axis < 3 && chain_on_axis[axis] < 0;
    }
};
```

### 4.5 Chain

```cpp
struct Chain {
    int id = -1;                            // unique chain ID (0-indexed)
    std::string name;                       // display name ("Chain A", "Chain B", ...)
    float color[4] = {1, 1, 1, 1};         // RGBA display color
    std::vector<IVec3> blocks;              // ordered list of grid positions in this chain
    SpawnFace head_direction = FACE_POS_X;  // current placement direction at the head

    // Undo stack: each entry is the number of blocks added by one PlaceBlock() call.
    // Used by DeleteLastBlock() to atomically undo multi-junction placements in one press.
    // Not persisted to JSON — resets to empty after save/load.
    std::vector<int> undo_stack;

    // The head position (last block in the chain)
    IVec3 Head() const { return blocks.back(); }

    // Number of blocks in this chain
    int Size() const { return static_cast<int>(blocks.size()); }

    // Is this chain empty?
    bool Empty() const { return blocks.empty(); }
};
```

**Undo stack semantics:** When `PlaceBlock()` crosses N junction cells and lands on a free cell,
it pushes `N + 1` onto `undo_stack`. `DeleteLastBlock()` pops this count and removes all `N + 1`
blocks in one press, preventing intermediate "head-inside-occupied-cell" visual states. If
`undo_stack` is empty (initial block, or after load), `DeleteLastBlock()` removes exactly 1.

### 4.6 AppMode — Application State

```cpp
enum class AppMode {
    BUILD,                // Normal block placement mode
    NEW_CHAIN_PICK,       // Waiting for user to click an existing block for junction
    NEW_CHAIN_DIRECTION,  // Waiting for user to pick a direction from the junction block
    SIMULATE,             // Physics simulation running (Phase 2)
};
```

### 4.7 ChainWorld — The Central Data Structure

```cpp
struct ChainWorld {
    // --- Grid storage ---
    std::unordered_map<IVec3, GridCell, IVec3Hash> grid;

    // --- Chains ---
    std::vector<Chain> chains;
    int active_chain_id = -1;  // which chain is currently being extended

    // --- Global parameters ---
    double bead_size = 0.05;    // cube edge length in meters (range 0.01–0.10)
    double gap_ratio = 0.05;    // gap as fraction of bead_size

    // --- Derived ---
    double CellStride() const { return bead_size * (1.0 + gap_ratio); }
    double Gap() const { return bead_size * gap_ratio; }
    double HalfSize() const { return bead_size / 2.0; }

    // Convert grid coords to world position (meters)
    void GridToWorld(const IVec3& g, double out[3]) const {
        double s = CellStride();
        out[0] = g.x * s;
        out[1] = g.y * s;
        out[2] = g.z * s + bead_size;  // lift off floor by one bead_size
    }

    // --- Active chain convenience ---
    Chain* ActiveChain() {
        if (active_chain_id >= 0 && active_chain_id < (int)chains.size())
            return &chains[active_chain_id];
        return nullptr;
    }
    const Chain* ActiveChain() const {
        if (active_chain_id >= 0 && active_chain_id < (int)chains.size())
            return &chains[active_chain_id];
        return nullptr;
    }

    // --- Chain color palette (auto-assigned) ---
    static constexpr float kPalette[][4] = {
        {0.2f, 0.6f, 1.0f, 1.0f},  // blue
        {1.0f, 0.3f, 0.3f, 1.0f},  // red
        {0.3f, 0.9f, 0.3f, 1.0f},  // green
        {1.0f, 0.8f, 0.2f, 1.0f},  // yellow
        {0.8f, 0.3f, 0.8f, 1.0f},  // purple
        {1.0f, 0.5f, 0.0f, 1.0f},  // orange
        {0.0f, 0.8f, 0.8f, 1.0f},  // cyan
        {0.9f, 0.5f, 0.6f, 1.0f},  // pink
    };
    static constexpr int kPaletteSize = 8;
};
```

### 4.8 AppState — Global Application State

```cpp
struct AppState {
    // --- Data ---
    ChainWorld world;
    AppMode mode = AppMode::BUILD;

    // --- New chain workflow ---
    IVec3 junction_pick;          // grid pos of selected junction block
    bool junction_picked = false; // true after user clicks a block in NEW_CHAIN_PICK mode

    // --- MuJoCo visualization (Build Stage) ---
    mjvCamera cam;
    mjvOption opt;
    mjvScene  scn;
    mjrContext con;

    // --- MuJoCo physics (Simulation Stage, Phase 2) ---
    mjModel* sim_model = nullptr;
    mjData*  sim_data  = nullptr;
    mjSpec*  sim_spec  = nullptr;

    // --- UI ---
    mjUI     ui;
    mjuiState uistate;
    int ui_width = 220;  // right panel width in pixels

    // --- Mouse ---
    bool mouse_left   = false;
    bool mouse_middle = false;
    bool mouse_right  = false;
    double mouse_lastx = 0;
    double mouse_lasty = 0;

    // --- Status text ---
    char status_text[512] = "";

    // --- File I/O ---
    char io_filename[256] = "";
    int  io_mode = 0;  // 0=none, 1=save, 2=load

    // --- Profiler ---
    bool show_profiler = false;   // toggled by P key in SIMULATE mode

    // --- Video Recording ---
    bool  is_recording   = false;
    std::string record_path;      // auto-timestamped .mp4 filename
    FILE* record_pipe    = nullptr;  // ffmpeg stdin pipe (popen)
    int   record_width   = 0;
    int   record_height  = 0;
    std::vector<unsigned char> record_pixels;  // raw RGB scratch buffer
};
```

**CRITICAL:** `AppState` is the single global instance that owns everything. Declare it as
a global in `main.cc`:

```cpp
// main.cc — top of file
static AppState g_app;
```

All other files access it via `extern AppState& g_app;` declared in their headers, or
receive the relevant sub-objects as function parameters.

---

## 5. Application Lifecycle

### 5.1 main.cc — Entry Point

```
main(argc, argv):
    1. Initialize AppState
       - world.bead_size = 0.05
       - world.gap_ratio = 0.05
       - mode = BUILD
    2. glfwInit()
    3. glfwCreateWindow(1400, 900, "ChainMaker")
    4. glfwMakeContextCurrent(window)
    5. glfwSwapInterval(1)
    6. Initialize MuJoCo visualization (NO model needed):
       - mjv_defaultCamera(&cam)
       - mjv_defaultOption(&opt)
       - mjv_defaultScene(&scn)
       - mjr_defaultContext(&con)
       - mjv_makeScene(NULL, &scn, 20000)   // NULL model, 20000 geom capacity
       - mjr_makeContext(NULL, &con, mjFONTSCALE_150)  // NULL model
    7. Build UI panel (§7)
    8. Create first chain with one block at origin:
       - CreateFirstChain(world)
    9. Set GLFW callbacks:
       - glfwSetKeyCallback(window, KeyboardCallback)
       - glfwSetCursorPosCallback(window, MouseMoveCallback)
       - glfwSetMouseButtonCallback(window, MouseButtonCallback)
       - glfwSetScrollCallback(window, ScrollCallback)
   10. Main loop:
       while (!glfwWindowShouldClose(window)):
           if (mode == SIMULATE):
               StepSimulation(sim_model, sim_data)  // Phase 2
               mjv_updateScene(sim_model, sim_data, &opt, NULL, &cam, mjCAT_ALL, &scn)
           else:
               PopulateBuildScene(world, scn, cam)  // §6
           
           glfwGetFramebufferSize(window, &width, &height)
           mjrRect viewport = {0, 0, width, height}
           
           // Update UI layout
           SetupUIRect(viewport)
           mjui_update(-1, -1, &ui, &uistate, &con)
           
           // Render
           mjr_render(viewport, &scn, &con)
           RenderOverlays(viewport, con)        // status text
           mjui_render(&ui, &uistate, &con)     // UI panel
           
           glfwSwapBuffers(window)
           glfwPollEvents()
   11. Cleanup:
       - mjv_freeScene(&scn)
       - mjr_freeContext(&con)
       - if (sim_data) mj_deleteData(sim_data)
       - if (sim_model) mj_deleteModel(sim_model)
       - if (sim_spec) mj_deleteSpec(sim_spec)
       - glfwTerminate()
```

### 5.2 Key Insight: No mjModel During Build Stage

During the Build Stage, `mjModel* m` is **NULL**. MuJoCo's visualization functions
(`mjv_makeScene`, `mjr_makeContext`) accept NULL for the model parameter. We populate
`mjvScene` manually using `mjv_initGeom()`. This is what makes block placement O(1).

### 5.3 Scene Capacity

`mjv_makeScene(NULL, &scn, 20000)` — allocate space for 20,000 visual geoms. At 1000
blocks plus floor, ghost, markers, and arrows, this is more than enough. If
`scn.ngeom >= scn.maxgeom`, the scene silently drops geoms (check `scn.ngeom` vs
`scn.maxgeom` and warn).

---

## 6. Rendering Pipeline — Build Stage

### 6.1 PopulateBuildScene() — Called Every Frame

This function lives in `chainmaker_render.cpp`:

```cpp
void PopulateBuildScene(const ChainWorld& world, mjvScene& scn,
                        const mjvCamera& cam, AppMode mode,
                        const IVec3* junction_pick) {
    scn.ngeom = 0;  // reset scene

    // 1. Floor plane
    AddFloorGeom(scn, world);

    // 2. All occupied grid cells as colored cubes
    for (const auto& [pos, cell] : world.grid) {
        AddBlockGeom(scn, world, pos, cell);
    }

    // 3. Ghost preview (semi-transparent) at next placement position
    if (mode == AppMode::BUILD) {
        const Chain* active = world.ActiveChain();
        if (active && !active->Empty()) {
            AddGhostGeom(scn, world, *active);
        }
    }

    // 4. Head marker (small sphere on active chain's head)
    if (mode == AppMode::BUILD || mode == AppMode::NEW_CHAIN_PICK) {
        const Chain* active = world.ActiveChain();
        if (active && !active->Empty()) {
            AddHeadMarker(scn, world, active->Head());
        }
    }

    // 5. Direction arrow from head
    if (mode == AppMode::BUILD) {
        const Chain* active = world.ActiveChain();
        if (active && !active->Empty()) {
            AddDirectionArrow(scn, world, active->Head(), active->head_direction);
        }
    }

    // 6. Junction highlight (during new-chain-pick mode)
    if (mode == AppMode::NEW_CHAIN_PICK && junction_pick) {
        AddJunctionHighlight(scn, world, *junction_pick);
    }
}
```

### 6.2 mjv_initGeom() — The Core Rendering Primitive

**Signature** (from `src/engine/engine_vis_visualize.c:305`):
```c
void mjv_initGeom(mjvGeom* geom, int type, const mjtNum* size,
                  const mjtNum* pos, const mjtNum* mat, const float* rgba);
```

**Parameters:**
- `geom`: pointer to `scn.geoms[scn.ngeom]` — write into the next slot
- `type`: `mjGEOM_BOX`, `mjGEOM_PLANE`, `mjGEOM_SPHERE`, `mjGEOM_ARROW`, `mjGEOM_LINEBOX`
- `size`: half-extents for BOX (`{hx, hy, hz}`), extents for PLANE, radius for SPHERE
- `pos`: world position `{x, y, z}`. If NULL, defaults to origin.
- `mat`: 3×3 rotation matrix (row-major, 9 doubles). If NULL, defaults to identity.
- `rgba`: color `{r, g, b, a}` as floats 0–1. If NULL, defaults to gray.

**Usage pattern:**
```cpp
void AddBlockGeom(mjvScene& scn, const ChainWorld& world,
                  const IVec3& pos, const GridCell& cell) {
    if (scn.ngeom >= scn.maxgeom) return;  // overflow guard

    mjvGeom* g = &scn.geoms[scn.ngeom];
    double half = world.HalfSize();
    mjtNum size[3] = {half, half, half};
    mjtNum wpos[3];
    world.GridToWorld(pos, wpos);

    // Determine color: blend chain colors if junction
    float rgba[4];
    ComputeBlockColor(world, cell, rgba);

    mjv_initGeom(g, mjGEOM_BOX, size, wpos, NULL, rgba);

    // Tag the geom for picking (optional — use objtype/objid/dataid)
    g->objtype = mjOBJ_UNKNOWN;  // custom marker
    g->dataid = EncodeGridPos(pos);  // encode position for pick decoding

    scn.ngeom++;
}
```

### 6.3 Floor Geom

```cpp
void AddFloorGeom(mjvScene& scn, const ChainWorld& world) {
    if (scn.ngeom >= scn.maxgeom) return;
    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjtNum size[3] = {10.0, 10.0, 0.02};  // large plane
    mjtNum pos[3]  = {0, 0, 0};           // at z=0
    float rgba[4]  = {0.4f, 0.4f, 0.45f, 1.0f};  // dark gray
    mjv_initGeom(g, mjGEOM_PLANE, size, pos, NULL, rgba);
    scn.ngeom++;
}
```

### 6.4 Ghost Preview

The ghost shows where the next block will be placed. It uses the active chain's head
position + direction offset:

```cpp
void AddGhostGeom(mjvScene& scn, const ChainWorld& world, const Chain& chain) {
    if (scn.ngeom >= scn.maxgeom) return;

    IVec3 ghost_pos = chain.Head() + FaceToOffset(chain.head_direction);
    mjtNum wpos[3];
    world.GridToWorld(ghost_pos, wpos);

    double half = world.HalfSize();
    mjtNum size[3] = {half, half, half};
    float rgba[4] = {chain.color[0], chain.color[1], chain.color[2], 0.3f};  // semi-transparent

    // Use LINEBOX (wireframe) for ghost
    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjv_initGeom(g, mjGEOM_LINEBOX, size, wpos, NULL, rgba);
    scn.ngeom++;
}
```

### 6.5 Head Marker (Small Sphere)

```cpp
void AddHeadMarker(mjvScene& scn, const ChainWorld& world, const IVec3& head) {
    if (scn.ngeom >= scn.maxgeom) return;
    mjtNum wpos[3];
    world.GridToWorld(head, wpos);
    mjtNum size[3] = {world.HalfSize() * 0.3, 0, 0};  // small sphere
    float rgba[4] = {1.0f, 1.0f, 0.0f, 0.8f};         // yellow
    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjv_initGeom(g, mjGEOM_SPHERE, size, wpos, NULL, rgba);
    scn.ngeom++;
}
```

### 6.6 Direction Arrow

```cpp
void AddDirectionArrow(mjvScene& scn, const ChainWorld& world,
                       const IVec3& head, SpawnFace face) {
    if (scn.ngeom >= scn.maxgeom) return;

    mjtNum wpos[3];
    world.GridToWorld(head, wpos);

    // Offset the arrow start to the face of the block
    int axis, sign;
    FaceToAxisSign(face, axis, sign);
    wpos[axis] += sign * world.HalfSize();

    // Arrow size: [shaft_length, shaft_radius, head_size]
    mjtNum size[3] = {world.CellStride() * 0.6, world.HalfSize() * 0.1, 0};
    float rgba[4] = {1.0f, 1.0f, 1.0f, 0.7f};  // white semi-transparent

    // Build rotation matrix to orient arrow along the face direction
    mjtNum mat[9] = {1,0,0, 0,1,0, 0,0,1};  // identity
    BuildArrowRotation(face, mat);  // set mat to rotate Z-axis to face direction

    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjv_initGeom(g, mjGEOM_ARROW, size, wpos, mat, rgba);
    scn.ngeom++;
}
```

### 6.7 Block Color Computation

When a block is at a junction (multiple chains), blend the colors:

```cpp
void ComputeBlockColor(const ChainWorld& world, const GridCell& cell, float rgba[4]) {
    int count = 0;
    float r = 0, g = 0, b = 0;
    for (int ax = 0; ax < 3; ax++) {
        int cid = cell.chain_on_axis[ax];
        if (cid >= 0 && cid < (int)world.chains.size()) {
            r += world.chains[cid].color[0];
            g += world.chains[cid].color[1];
            b += world.chains[cid].color[2];
            count++;
        }
    }
    if (count > 0) {
        rgba[0] = r / count;
        rgba[1] = g / count;
        rgba[2] = b / count;
        rgba[3] = 1.0f;
    } else {
        rgba[0] = rgba[1] = rgba[2] = 0.5f;
        rgba[3] = 1.0f;
    }
}
```

### 6.8 Camera Control

Use MuJoCo's built-in camera functions. In the mouse callbacks:

```cpp
// Mouse drag → camera orbit/pan
mjv_moveCamera(NULL, action, dx/height, dy/height, &scn, &cam);

// action values:
//   mjMOUSE_ROTATE_V / mjMOUSE_ROTATE_H  — left-drag
//   mjMOUSE_MOVE_V / mjMOUSE_MOVE_H      — right-drag
//   mjMOUSE_ZOOM                          — scroll
```

**Important:** The first parameter (model) can be NULL during build stage. `mjv_moveCamera`
only uses the model for perturbation, which we don't need.

---

## 7. UI Construction

### 7.1 UI Panel Setup

The UI panel is built using MuJoCo's `mjUI` widget system. It renders on the right side of
the viewport.

```cpp
// In chainmaker_ui.cpp

void BuildUI(AppState& app) {
    std::memset(&app.uistate, 0, sizeof(mjuiState));
    std::memset(&app.ui, 0, sizeof(mjUI));

    app.ui.spacing  = mjui_themeSpacing(1);   // compact spacing
    app.ui.color    = mjui_themeColor(1);      // dark theme
    app.ui.predicate = nullptr;
    app.ui.rectid   = 1;
    app.ui.auxid    = 0;
    app.ui.radiocol = 1;

    // Custom button color (blue tint)
    app.ui.color.button[0] = 0.15f;
    app.ui.color.button[1] = 0.55f;
    app.ui.color.button[2] = 0.95f;

    // Build the control definitions
    BuildControlSection(app);

    mjui_resize(&app.ui, &app.con);
    mjr_addAux(app.ui.auxid, app.ui.width, app.ui.maxheight,
               app.ui.spacing.samples, &app.con);
}
```

### 7.2 mjuiDef Array — Control Section

Each UI element is defined by an `mjuiDef` entry. The array must end with `mjITEM_END`.

```cpp
// Button label constants (for matching in click handlers)
static const char* kBtnPlace     = "Place Block";
static const char* kBtnDelete    = "Delete Block";
static const char* kBtnNewChain  = "New Chain";
static const char* kBtnSave      = "Save";
static const char* kBtnLoad      = "Load";
static const char* kBtnSimulate  = "Simulate";

void BuildControlSection(AppState& app) {
    // Maximum 30 UI items
    mjuiDef def[30];
    int di = 0;

    // --- Section header ---
    std::memset(&def[di], 0, sizeof(mjuiDef));
    def[di].type = mjITEM_SECTION;
    std::snprintf(def[di].name, sizeof(def[di].name), "Chain Controls");
    def[di].state = 1;  // open by default
    di++;

    // --- Buttons ---
    auto addButton = [&](const char* label) {
        std::memset(&def[di], 0, sizeof(mjuiDef));
        def[di].type = mjITEM_BUTTON;
        std::snprintf(def[di].name, sizeof(def[di].name), "%s", label);
        def[di].state = 2;  // enabled
        di++;
    };

    addButton(kBtnPlace);
    addButton(kBtnDelete);
    addButton(kBtnNewChain);

    // --- Separator ---
    std::memset(&def[di], 0, sizeof(mjuiDef));
    def[di].type = mjITEM_SEPARATOR;
    di++;

    // --- Active chain selector (static text, updated per-frame) ---
    // Use mjITEM_STATIC for read-only display
    std::memset(&def[di], 0, sizeof(mjuiDef));
    def[di].type = mjITEM_STATIC;
    std::snprintf(def[di].name, sizeof(def[di].name), "Active Chain");
    def[di].state = 1;
    di++;

    // --- Bead size slider ---
    std::memset(&def[di], 0, sizeof(mjuiDef));
    def[di].type = mjITEM_SLIDERNUM;
    std::snprintf(def[di].name, sizeof(def[di].name), "Bead Size");
    def[di].state = 2;
    def[di].pdata = &app.world.bead_size;
    std::snprintf(def[di].other, sizeof(def[di].other), "0.01 0.10");  // min max
    di++;

    // --- Gap ratio slider ---
    std::memset(&def[di], 0, sizeof(mjuiDef));
    def[di].type = mjITEM_SLIDERNUM;
    std::snprintf(def[di].name, sizeof(def[di].name), "Gap Ratio");
    def[di].state = 2;
    def[di].pdata = &app.world.gap_ratio;
    std::snprintf(def[di].other, sizeof(def[di].other), "0.0 0.5");  // min max
    di++;

    // --- Separator ---
    std::memset(&def[di], 0, sizeof(mjuiDef));
    def[di].type = mjITEM_SEPARATOR;
    di++;

    // --- Save/Load ---
    addButton(kBtnSave);
    addButton(kBtnLoad);

    // --- Separator ---
    std::memset(&def[di], 0, sizeof(mjuiDef));
    def[di].type = mjITEM_SEPARATOR;
    di++;

    // --- Simulate ---
    addButton(kBtnSimulate);

    // --- Terminator ---
    std::memset(&def[di], 0, sizeof(mjuiDef));
    def[di].type = mjITEM_END;

    mjui_add(&app.ui, def);
}
```

### 7.3 UI Rect Setup (Per Frame)

The `uistate` requires a **two-rect layout** matching MuJoCo's `simulate` pattern:
- `rect[0]` = full framebuffer viewport (required as coordinate base by `mjr_findRect`)
- `rect[1]` = UI panel rect (referenced by `ui.rectid = 1`)

```cpp
void SetupUIRect(AppState& app, mjrRect viewport) {
    // ui.rectid = 1 is set in BuildUI — keep it, don't override.
    app.uistate.nrect = 2;

    int uiw = app.ui.width;
    if (uiw < app.ui_width) uiw = app.ui_width;

    // rect[0]: full framebuffer (required for coordinate mapping)
    app.uistate.rect[0] = {0, 0, viewport.width, viewport.height};
    // rect[1]: UI panel on the right edge
    app.uistate.rect[1] = {viewport.width - uiw, 0, uiw, viewport.height};
}
```

**Critical:** `ui.rectid` must be `1` (set in `BuildUI`). Using `rectid = 0` with a single-rect
layout causes `mjui_event` to miscompute item positions. Call `SetupUIRect` + `mjui_resize`
once after `BuildUI` (with valid framebuffer dims) and again each frame in the render loop.

### 7.4 Button Click Handling

`mjui_event` requires `uistate.button` to be set to the correct `mjtButton` enum value before
it is called. If `button = 0 (mjBUTTON_NONE)`, `mjui_event` exits immediately with NULL
because its first check is `if (button != mjBUTTON_LEFT) return NULL`.

```cpp
void HandleUIClick(AppState& app, mjuiItem* item, int glfw_action) {
    if (glfw_action != GLFW_PRESS) return;
    if (item->type != mjITEM_BUTTON) return;

    if (std::strcmp(item->name, kBtnPlace) == 0)    PlaceBlock(app.world);
    else if (std::strcmp(item->name, kBtnDelete) == 0)  DeleteLastBlock(app.world);
    else if (std::strcmp(item->name, kBtnNewChain) == 0) app.mode = AppMode::NEW_CHAIN_PICK;
    else if (std::strcmp(item->name, kBtnSave) == 0)  { app.io_mode = 1; app.io_filename[0] = '\0'; }
    else if (std::strcmp(item->name, kBtnLoad) == 0)  { app.io_mode = 2; app.io_filename[0] = '\0'; }
    else if (std::strcmp(item->name, kBtnSimulate) == 0) EnterSimulation(app);
}
```

### 7.5 Status Bar Overlay

```cpp
void RenderOverlays(AppState& app, mjrRect viewport) {
    const Chain* active = app.world.ActiveChain();
    if (active) {
        std::snprintf(app.status_text, sizeof(app.status_text),
            "%s | Head: (%d,%d,%d) | Dir: %s | %d blocks | %zu total",
            active->name.c_str(),
            active->Head().x, active->Head().y, active->Head().z,
            FaceName(active->head_direction),
            active->Size(),
            app.world.grid.size());
    }
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                app.status_text, NULL, &app.con);

    // File I/O prompt
    if (app.io_mode != 0) {
        char io_text[320];
        const char* mode_str = (app.io_mode == 1) ? "Save" : "Load";
        std::snprintf(io_text, sizeof(io_text),
            "%s filename: %s (Enter=OK, Esc=Cancel)", mode_str, app.io_filename);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                    io_text, NULL, &app.con);
    }
}
```

---

## 8. Keyboard & Mouse Handling

### 8.1 Keyboard Callback

```cpp
void KeyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
    AppState& app = g_app;

    // Let UI consume first
    if (mjui_event(&app.ui, &app.uistate, &app.con)) return;

    // File I/O capture mode
    if (app.io_mode != 0 && act == GLFW_PRESS) {
        HandleFileIOKey(app, key, mods);
        return;
    }

    if (act != GLFW_PRESS) return;

    // Only handle build-mode keys when in BUILD mode
    if (app.mode == AppMode::BUILD) {
        switch (key) {
            case GLFW_KEY_C:
                PlaceBlock(app.world);
                break;
            case GLFW_KEY_DELETE:
                DeleteLastBlock(app.world);
                break;
            case GLFW_KEY_N:
                app.mode = AppMode::NEW_CHAIN_PICK;
                break;

            // Direction keys
            case GLFW_KEY_RIGHT:
                SetDirection(app.world, FACE_POS_X);
                break;
            case GLFW_KEY_LEFT:
                SetDirection(app.world, FACE_NEG_X);
                break;
            case GLFW_KEY_UP:
                SetDirection(app.world, FACE_POS_Y);
                break;
            case GLFW_KEY_DOWN:
                SetDirection(app.world, FACE_NEG_Y);
                break;
            case GLFW_KEY_Z:
                SetDirection(app.world, FACE_POS_Z);
                break;
            case GLFW_KEY_X:
                SetDirection(app.world, FACE_NEG_Z);
                break;

            // Chain switching (1-9)
            case GLFW_KEY_1: case GLFW_KEY_2: case GLFW_KEY_3:
            case GLFW_KEY_4: case GLFW_KEY_5: case GLFW_KEY_6:
            case GLFW_KEY_7: case GLFW_KEY_8: case GLFW_KEY_9:
                SwitchChain(app.world, key - GLFW_KEY_1);
                break;

            // Gap control
            case GLFW_KEY_EQUAL:     // '+' (shares '=')
            case GLFW_KEY_KP_ADD:
                AdjustGap(app.world, +0.01);
                break;
            case GLFW_KEY_MINUS:
            case GLFW_KEY_KP_SUBTRACT:
                AdjustGap(app.world, -0.01);
                break;

            // Save/Load shortcuts
            case GLFW_KEY_S:
                if (mods & GLFW_MOD_CONTROL) {
                    app.io_mode = 1; app.io_filename[0] = '\0';
                }
                break;
            case GLFW_KEY_L:
                if (mods & GLFW_MOD_CONTROL) {
                    app.io_mode = 2; app.io_filename[0] = '\0';
                }
                break;

            // Simulate
            case GLFW_KEY_P:
                EnterSimulation(app);
                break;
        }
    }
    else if (app.mode == AppMode::SIMULATE) {
        switch (key) {
            case GLFW_KEY_R:
                ExitSimulation(app);  // return to build mode
                break;
            case GLFW_KEY_P:
                app.show_profiler = !app.show_profiler;  // toggle full profiler overlay
                break;
            case GLFW_KEY_F9:
                ToggleRecording(app);  // start / stop video recording
                break;
        }
    }
    else if (app.mode == AppMode::NEW_CHAIN_PICK) {
        if (key == GLFW_KEY_ESCAPE) {
            app.mode = AppMode::BUILD;  // cancel
        }
    }
}
```

### 8.2 Mouse Button Callback

Three rules must all be followed for UI clicks to work:

1. **Set `uistate.button`** to the correct `mjtButton` enum (`mjui_event` exits immediately if `button != mjBUTTON_LEFT`).
2. **Scale cursor to framebuffer coords** — `glfwGetCursorPos` returns window-logical pixels; on HiDPI displays these differ from framebuffer pixels. Multiply by `fbw / winw`.
3. **Set `uistate.mouserect`** via `mjr_findRect` so slider dragging (`dragrect`) works across move events.

```cpp
void MouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
    AppState& app = g_app;

    app.mouse_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    app.mouse_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    app.mouse_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);

    double cx, cy;
    glfwGetCursorPos(window, &cx, &cy);

    int fbw, fbh, winw, winh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glfwGetWindowSize(window, &winw, &winh);

    // Scale cursor from window coords to framebuffer coords (handles HiDPI)
    double cx_fb = (winw > 0) ? cx * fbw / winw : cx;
    double cy_fb = (winh > 0) ? cy * fbh / winh : cy;

    app.mouse_lastx = cx_fb;
    app.mouse_lasty = cy_fb;

    // Translate GLFW button → mjtButton (critical: mjui_event checks uistate.button)
    mjtButton mj_button = mjBUTTON_NONE;
    if (button == GLFW_MOUSE_BUTTON_LEFT)        mj_button = mjBUTTON_LEFT;
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)  mj_button = mjBUTTON_RIGHT;
    else if (button == GLFW_MOUSE_BUTTON_MIDDLE) mj_button = mjBUTTON_MIDDLE;

    app.uistate.type    = (act == GLFW_PRESS) ? mjEVENT_PRESS : mjEVENT_RELEASE;
    app.uistate.button  = mj_button;       // ← required; mjui_event checks this
    app.uistate.left    = app.mouse_left;
    app.uistate.right   = app.mouse_right;
    app.uistate.middle  = app.mouse_middle;
    app.uistate.x       = cx_fb;
    app.uistate.y       = fbh - cy_fb;     // flip Y for OpenGL origin
    app.uistate.shift   = (mods & GLFW_MOD_SHIFT)   ? 1 : 0;
    app.uistate.control = (mods & GLFW_MOD_CONTROL) ? 1 : 0;
    app.uistate.alt     = (mods & GLFW_MOD_ALT)     ? 1 : 0;

    SetupUIRect(app, {0, 0, fbw, fbh});

    // mouserect: which rect does the cursor land in? (1 = UI panel, 0 = elsewhere)
    app.uistate.mouserect = mjr_findRect(
        (int)mju_round(cx_fb), (int)mju_round(app.uistate.y),
        app.uistate.nrect - 1, app.uistate.rect + 1) + 1;

    // Track drag rect (enables slider dragging in MouseMoveCallback)
    if (act == GLFW_PRESS && app.uistate.mouserect) {
        app.uistate.dragbutton = mj_button;
        app.uistate.dragrect   = app.uistate.mouserect;
    }

    // Dispatch to UI if event is for the UI rect
    bool for_ui = (app.uistate.dragrect == app.ui.rectid) ||
                  (app.uistate.dragrect == 0 && app.uistate.mouserect == app.ui.rectid);
    if (for_ui) {
        if (mjuiItem* changed = mjui_event(&app.ui, &app.uistate, &app.con)) {
            HandleUIClick(app, changed, act);
            if (act == GLFW_RELEASE) { app.uistate.dragrect = 0; app.uistate.dragbutton = 0; }
            return;
        }
    }

    if (act == GLFW_RELEASE) { app.uistate.dragrect = 0; app.uistate.dragbutton = 0; }

    // Block world interaction when cursor is in the UI panel
    if (app.uistate.mouserect == app.ui.rectid) return;

    // NEW_CHAIN_PICK: left-click picks a block
    if (app.mode == AppMode::NEW_CHAIN_PICK && act == GLFW_PRESS
        && button == GLFW_MOUSE_BUTTON_LEFT) {
        IVec3 hit;
        if (RayPickBlock(app, cx_fb, cy_fb, fbw, fbh, hit))
            StartNewChainFromBlock(app, hit);
    }
}
```

### 8.3 Mouse Move Callback (Camera)

```cpp
void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    AppState& app = g_app;

    int fbw, fbh, winw, winh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glfwGetWindowSize(window, &winw, &winh);

    // Scale to framebuffer coords
    double xfb = (winw > 0) ? xpos * fbw / winw : xpos;
    double yfb = (winh > 0) ? ypos * fbh / winh : ypos;

    double dx = xfb - app.mouse_lastx;
    double dy = yfb - app.mouse_lasty;
    app.mouse_lastx = xfb;
    app.mouse_lasty = yfb;

    if (!app.mouse_left && !app.mouse_middle && !app.mouse_right) return;

    app.uistate.type = mjEVENT_MOVE;
    app.uistate.x    = xfb;
    app.uistate.y    = fbh - yfb;

    // Update mouserect
    app.uistate.mouserect = mjr_findRect(
        (int)mju_round(xfb), (int)mju_round(app.uistate.y),
        app.uistate.nrect - 1, app.uistate.rect + 1) + 1;

    // Route to UI if dragging within UI panel (slider drag keeps dragrect set)
    if (app.uistate.dragrect == app.ui.rectid) {
        mjui_event(&app.ui, &app.uistate, &app.con);
        return;
    }

    // Block world camera if cursor is in the UI panel (not dragging)
    if (app.uistate.mouserect == app.ui.rectid) return;

    mjtMouse action;
    if      (app.mouse_left)   action = mjMOUSE_ROTATE_V;
    else if (app.mouse_right)  action = mjMOUSE_MOVE_H;
    else                       action = mjMOUSE_ZOOM;

    mjv_moveCamera(NULL, action, dx / fbh, dy / fbh, &app.scn, &app.cam);
}
```

### 8.4 Scroll Callback (Zoom)

```cpp
void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    AppState& app = g_app;

    double cx, cy;
    glfwGetCursorPos(window, &cx, &cy);

    int fbw, fbh, winw, winh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glfwGetWindowSize(window, &winw, &winh);

    double xfb = (winw > 0) ? cx * fbw / winw : cx;
    double yfb = (winh > 0) ? cy * fbh / winh : cy;

    // Determine if cursor is in the UI panel
    int mouserect = mjr_findRect(
        (int)mju_round(xfb), (int)mju_round(fbh - yfb),
        app.uistate.nrect - 1, app.uistate.rect + 1) + 1;

    if (mouserect == app.ui.rectid) return;  // don't zoom when hovering UI panel

    mjv_moveCamera(NULL, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &app.scn, &app.cam);
}
```

### 8.5 File I/O Key Handler

Captures filename character-by-character (same pattern as progui):

```cpp
void HandleFileIOKey(AppState& app, int key, int mods) {
    if (key == GLFW_KEY_ESCAPE) {
        app.io_mode = 0;  // cancel
        return;
    }
    if (key == GLFW_KEY_ENTER) {
        if (app.io_mode == 1) {
            SaveWorldToJSON(app.world, app.io_filename);
        } else if (app.io_mode == 2) {
            LoadWorldFromJSON(app.world, app.io_filename);
        }
        app.io_mode = 0;
        return;
    }
    if (key == GLFW_KEY_BACKSPACE) {
        size_t len = std::strlen(app.io_filename);
        if (len > 0) app.io_filename[len - 1] = '\0';
        return;
    }
    // Append alphanumeric and filename-safe characters
    char c = '\0';
    if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z) {
        c = (mods & GLFW_MOD_SHIFT) ? ('A' + key - GLFW_KEY_A) : ('a' + key - GLFW_KEY_A);
    } else if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9) {
        c = '0' + key - GLFW_KEY_0;
    } else if (key == GLFW_KEY_PERIOD) c = '.';
    else if (key == GLFW_KEY_MINUS) c = '-';
    else if (key == GLFW_KEY_SLASH) c = '/';
    else if (key == GLFW_KEY_BACKSLASH) c = '\\';

    if (c != '\0') {
        size_t len = std::strlen(app.io_filename);
        if (len + 1 < sizeof(app.io_filename)) {
            app.io_filename[len] = c;
            app.io_filename[len + 1] = '\0';
        }
    }
}
```

### 8.6 UI Event Routing Rules

This subsection documents the state machine that routes input events to the UI panel or
the 3D world camera, and why each rule is required.

**`uistate.button` must be set (critical)**

`mjui_event` has this early-exit at the top:
```c
if ((type == PRESS || type == MOVE || type == RELEASE) && button != mjBUTTON_LEFT)
    return NULL;
```
GLFW button codes are 0/1/2. MuJoCo's `mjtButton` enum is `NONE=0, LEFT=1, RIGHT=2, MIDDLE=3`.
Button 0 = GLFW_MOUSE_BUTTON_LEFT, but `mjBUTTON_NONE = 0`. You must explicitly translate:
```cpp
mjtButton mj_button = (button == GLFW_MOUSE_BUTTON_LEFT)   ? mjBUTTON_LEFT   :
                      (button == GLFW_MOUSE_BUTTON_RIGHT)  ? mjBUTTON_RIGHT  :
                      (button == GLFW_MOUSE_BUTTON_MIDDLE) ? mjBUTTON_MIDDLE : mjBUTTON_NONE;
app.uistate.button = mj_button;
```

**Two-rect layout**

```
rect[0] = {0, 0, fbw, fbh}                     // full framebuffer (coordinate base)
rect[1] = {fbw - uiw, 0, uiw, fbh}             // UI panel
ui.rectid = 1                                   // items live in rect[1]
```
`mjr_findRect(x, y, nrect-1, rect+1)` searches `rect[1..nrect-1]` and returns 0-based index
or -1. Adding 1 converts to the outer index: 1 = in UI panel, 0 = not in panel.

**`mouserect` and `dragrect` lifecycle**

| Event | Action |
|-------|--------|
| PRESS in rect[1] | `dragrect = mouserect, dragbutton = button` |
| MOVE | if `dragrect == ui.rectid` → route to UI (slider drag continues even if cursor leaves) |
| RELEASE | clear `dragrect = 0, dragbutton = 0` |

**Camera blocking**

On PRESS or MOVE events: if `mouserect == ui.rectid` and no drag is active, the event
is consumed by the UI (or silently ignored), never forwarded to `mjv_moveCamera`.

---

## 9. Chain Operations

All chain mutation functions live in `chainmaker.cpp`. They operate on `ChainWorld` and
are pure logic (no rendering, no MuJoCo calls).

### 9.1 CreateFirstChain()

Called once at startup to create the initial chain with one block at the origin:

```cpp
void CreateFirstChain(ChainWorld& world) {
    Chain chain;
    chain.id = 0;
    chain.name = "Chain A";
    std::memcpy(chain.color, ChainWorld::kPalette[0], sizeof(float) * 4);
    chain.head_direction = FACE_POS_X;

    IVec3 origin{0, 0, 0};
    chain.blocks.push_back(origin);

    // Add to grid
    GridCell cell;
    cell.pos = origin;
    int axis, sign;
    FaceToAxisSign(chain.head_direction, axis, sign);
    cell.chain_on_axis[axis] = chain.id;
    world.grid[origin] = cell;

    world.chains.push_back(chain);
    world.active_chain_id = 0;
}
```

### 9.2 PlaceBlock()

Extends the active chain by one block in the current direction. If the next cell(s) in the
chain are already occupied (junctions), the chain walks through them and lands on the first
free cell, adding junction registrations along the way. All added blocks (N junctions + 1 new)
are recorded in `undo_stack` so `DeleteLastBlock` can remove them atomically.

```cpp
enum class PlaceResult {
    SUCCESS,
    NO_ACTIVE_CHAIN,
    TARGET_OCCUPIED_INCOMPATIBLE,  // turn block or axis already used
    WOULD_REVERSE,                 // 180° reversal not allowed
    HEAD_WOULD_BECOME_INVALID_TURN,// head is junction, can't be turn
};

PlaceResult PlaceBlock(ChainWorld& world) {
    Chain* chain = world.ActiveChain();
    if (!chain || chain->Empty()) return PlaceResult::NO_ACTIVE_CHAIN;

    SpawnFace dir = chain->head_direction;
    int target_axis, target_sign;
    FaceToAxisSign(dir, target_axis, target_sign);

    // --- Walk through occupied cells to find the next free landing cell ---
    std::vector<IVec3> junctions;
    IVec3 cursor = chain->Head();
    while (true) {
        // No 180° reversal
        if (chain->Size() + (int)junctions.size() >= 2) {
            IVec3 prev = junctions.empty()
                ? chain->blocks[chain->Size() - 2]
                : (junctions.size() == 1 ? chain->Head() : junctions[junctions.size() - 2]);
            IVec3 back_dir = prev - cursor;
            if (FaceToOffset(dir) == back_dir) return PlaceResult::WOULD_REVERSE;
        }

        IVec3 next = cursor + FaceToOffset(dir);
        auto it = world.grid.find(next);
        if (it == world.grid.end()) {
            cursor = next;
            break;  // free cell — land here
        }
        GridCell& cell = it->second;
        if (cell.is_turn) return PlaceResult::TARGET_OCCUPIED_INCOMPATIBLE;
        if (!cell.IsAxisFree(target_axis)) return PlaceResult::TARGET_OCCUPIED_INCOMPATIBLE;

        // next is a junction — register this chain on that axis and keep walking
        cell.chain_on_axis[target_axis] = chain->id;
        junctions.push_back(next);
        cursor = next;
    }

    // Commit junction blocks to chain
    for (const IVec3& jp : junctions)
        chain->blocks.push_back(jp);

    // Handle turn at the last block before landing
    if (chain->Size() >= 2) {
        IVec3 head = chain->blocks[chain->blocks.size() - 1];  // after appending junctions
        IVec3 prev = chain->blocks[chain->blocks.size() - 2];
        IVec3 entry_vec = head - prev;
        IVec3 exit_vec  = FaceToOffset(dir);
        if (entry_vec != exit_vec) {
            GridCell& head_cell = world.grid[head];
            if (head_cell.IsJunction()) return PlaceResult::HEAD_WOULD_BECOME_INVALID_TURN;
            head_cell.is_turn = true;
            for (int ax = 0; ax < 3; ax++) {
                if ((&entry_vec.x)[ax] != 0) head_cell.turn_entry_axis = ax;
                if ((&exit_vec.x)[ax] != 0)  head_cell.turn_exit_axis  = ax;
            }
        }
    }

    // Add the landing cell
    GridCell cell;
    cell.pos = cursor;
    cell.chain_on_axis[target_axis] = chain->id;
    world.grid[cursor] = cell;
    chain->blocks.push_back(cursor);

    // Record how many blocks were added (for atomic undo)
    chain->undo_stack.push_back((int)junctions.size() + 1);

    return PlaceResult::SUCCESS;
}
```

### 9.3 DeleteLastBlock()

Atomically removes all blocks added by the last `PlaceBlock()` call by popping from
`undo_stack`. If the stack is empty (e.g. after save/load), removes exactly 1 block.
The un-turn operation is applied once to the new chain head after all removals.

```cpp
void DeleteLastBlock(ChainWorld& world) {
    Chain* chain = world.ActiveChain();
    if (!chain || chain->Size() <= 1) return;  // keep at least 1 block

    // Determine how many blocks to remove atomically
    int count = 1;
    if (!chain->undo_stack.empty()) {
        count = chain->undo_stack.back();
        chain->undo_stack.pop_back();
        count = std::min(count, chain->Size() - 1);  // never delete the anchor block
    }

    auto removeOne = [&]() {
        IVec3 last = chain->blocks.back();
        chain->blocks.pop_back();

        auto it = world.grid.find(last);
        if (it != world.grid.end()) {
            GridCell& cell = it->second;
            for (int ax = 0; ax < 3; ax++) {
                if (cell.chain_on_axis[ax] == chain->id)
                    cell.chain_on_axis[ax] = -1;
            }
            if (cell.ChainCount() == 0)
                world.grid.erase(it);
            else {
                cell.is_turn = false;
                cell.turn_entry_axis = -1;
                cell.turn_exit_axis  = -1;
            }
        }
    };

    for (int i = 0; i < count; i++) removeOne();

    // Un-turn the new head (the block before what was just removed)
    if (!chain->Empty()) {
        IVec3 new_head = chain->Head();
        auto hit = world.grid.find(new_head);
        if (hit != world.grid.end()) {
            hit->second.is_turn = false;
            hit->second.turn_entry_axis = -1;
            hit->second.turn_exit_axis  = -1;
        }
    }
}
```

### 9.4 SetDirection()

Changes the placement direction. If the new direction differs from the entry direction
at the head, the head becomes a turn block (validated later in PlaceBlock):

```cpp
void SetDirection(ChainWorld& world, SpawnFace new_face) {
    Chain* chain = world.ActiveChain();
    if (!chain || chain->Empty()) return;

    // Prevent 180° reversal
    if (chain->Size() >= 2) {
        IVec3 head = chain->Head();
        IVec3 prev = chain->blocks[chain->Size() - 2];
        IVec3 entry_dir = head - prev;
        IVec3 reverse = {-entry_dir.x, -entry_dir.y, -entry_dir.z};
        if (FaceToOffset(new_face) == reverse) return;  // silently reject
    }

    chain->head_direction = new_face;
}
```

### 9.5 SwitchChain()

```cpp
void SwitchChain(ChainWorld& world, int index) {
    if (index >= 0 && index < (int)world.chains.size()) {
        world.active_chain_id = index;
    }
}
```

### 9.6 StartNewChainFromBlock()

Called when the user clicks a block in `NEW_CHAIN_PICK` mode:

```cpp
bool StartNewChainFromBlock(AppState& app, const IVec3& junction_pos) {
    auto it = app.world.grid.find(junction_pos);
    if (it == app.world.grid.end()) return false;

    GridCell& cell = it->second;
    if (cell.is_turn) return false;  // can't branch from a turn block

    // Find an available axis
    int free_axis = -1;
    for (int ax = 0; ax < 3; ax++) {
        if (cell.IsAxisFree(ax)) {
            free_axis = ax;
            break;
        }
    }
    if (free_axis < 0) return false;  // no axes available

    // Create new chain
    int new_id = (int)app.world.chains.size();
    Chain chain;
    chain.id = new_id;

    // Auto-generate name: "Chain A", "Chain B", ...
    chain.name = "Chain ";
    chain.name += (char)('A' + (new_id % 26));
    if (new_id >= 26) chain.name += std::to_string(new_id / 26);

    // Assign color from palette
    int ci = new_id % ChainWorld::kPaletteSize;
    std::memcpy(chain.color, ChainWorld::kPalette[ci], sizeof(float) * 4);

    // Default direction: positive along the free axis
    chain.head_direction = AxisSignToFace(free_axis, +1);

    // The junction block is the first block of this chain
    chain.blocks.push_back(junction_pos);
    cell.chain_on_axis[free_axis] = new_id;

    app.world.chains.push_back(chain);
    app.world.active_chain_id = new_id;
    app.mode = AppMode::BUILD;

    return true;
}
```

### 9.7 Ray-Pick Block Selection

For clicking on blocks during `NEW_CHAIN_PICK` mode. Uses a simple ray-vs-AABB test
against the grid:

```cpp
bool RayPickBlock(const AppState& app, double mouse_x, double mouse_y, IVec3& out) {
    int fbw, fbh;
    // Get framebuffer size from the current context
    // (passed in or obtained from GLFW)

    // Compute ray from camera through pixel
    // MuJoCo provides: mjv_select() but it requires a model.
    // Instead, manually compute camera ray from cam.lookat, cam.azimuth,
    // cam.elevation, cam.distance and the pixel coordinates.

    // For each occupied grid cell, test ray-AABB intersection
    // Return the closest hit.

    double closest_t = 1e30;
    bool found = false;

    for (const auto& [pos, cell] : app.world.grid) {
        double wpos[3];
        app.world.GridToWorld(pos, wpos);
        double half = app.world.HalfSize();

        double t;
        if (RayAABBIntersect(ray_origin, ray_dir,
                             wpos[0]-half, wpos[1]-half, wpos[2]-half,
                             wpos[0]+half, wpos[1]+half, wpos[2]+half, t)) {
            if (t < closest_t) {
                closest_t = t;
                out = pos;
                found = true;
            }
        }
    }
    return found;
}
```

**Note:** Implementing `RayAABBIntersect()` is a standard slab method. Also implement
camera-ray computation from `mjvCamera` fields (`lookat`, `azimuth`, `elevation`,
`distance`).

Alternatively, use a simpler approach: `mjv_select()` works on `mjvScene` geoms when a
model is present. Since we don't have a model, the manual ray-AABB approach is required.

---

## 10. Save / Load (JSON)

### 10.1 JSON Schema

```json
{
    "version": 1,
    "bead_size": 0.05,
    "gap_ratio": 0.05,
    "chains": [
        {
            "id": 0,
            "name": "Chain A",
            "color": [0.2, 0.6, 1.0, 1.0],
            "head_direction": 0,
            "blocks": [
                {
                    "pos": [0, 0, 0],
                    "is_turn": false,
                    "turn_entry_axis": -1,
                    "turn_exit_axis": -1
                },
                {
                    "pos": [1, 0, 0],
                    "is_turn": false,
                    "turn_entry_axis": -1,
                    "turn_exit_axis": -1
                },
                {
                    "pos": [1, 1, 0],
                    "is_turn": true,
                    "turn_entry_axis": 0,
                    "turn_exit_axis": 1
                }
            ]
        }
    ]
}
```

### 10.2 Implementation with nlohmann/json

```cpp
// chainmaker_io.h
#pragma once
#include "chainmaker.h"
#include <string>

bool SaveWorldToJSON(const ChainWorld& world, const char* filename);
bool LoadWorldFromJSON(ChainWorld& world, const char* filename);
```

```cpp
// chainmaker_io.cpp
#include "chainmaker_io.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

bool SaveWorldToJSON(const ChainWorld& world, const char* filename) {
    json j;
    j["version"] = 1;
    j["bead_size"] = world.bead_size;
    j["gap_ratio"] = world.gap_ratio;

    json chains_arr = json::array();
    for (const auto& chain : world.chains) {
        json cj;
        cj["id"] = chain.id;
        cj["name"] = chain.name;
        cj["color"] = {chain.color[0], chain.color[1], chain.color[2], chain.color[3]};
        cj["head_direction"] = static_cast<int>(chain.head_direction);

        json blocks_arr = json::array();
        for (const auto& pos : chain.blocks) {
            json bj;
            bj["pos"] = {pos.x, pos.y, pos.z};

            // Look up grid cell for turn info
            auto it = world.grid.find(pos);
            if (it != world.grid.end()) {
                bj["is_turn"] = it->second.is_turn;
                bj["turn_entry_axis"] = it->second.turn_entry_axis;
                bj["turn_exit_axis"] = it->second.turn_exit_axis;
            } else {
                bj["is_turn"] = false;
                bj["turn_entry_axis"] = -1;
                bj["turn_exit_axis"] = -1;
            }
            blocks_arr.push_back(bj);
        }
        cj["blocks"] = blocks_arr;
        chains_arr.push_back(cj);
    }
    j["chains"] = chains_arr;

    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "SaveWorldToJSON: cannot open " << filename << "\n";
        return false;
    }
    ofs << j.dump(2);  // pretty-print with 2-space indent
    return true;
}

bool LoadWorldFromJSON(ChainWorld& world, const char* filename) {
    std::ifstream ifs(filename);
    if (!ifs) {
        std::cerr << "LoadWorldFromJSON: cannot open " << filename << "\n";
        return false;
    }

    json j;
    try {
        ifs >> j;
    } catch (const json::parse_error& e) {
        std::cerr << "LoadWorldFromJSON: parse error: " << e.what() << "\n";
        return false;
    }

    // Clear existing data
    world.grid.clear();
    world.chains.clear();
    world.active_chain_id = -1;

    world.bead_size = j.value("bead_size", 0.05);
    world.gap_ratio = j.value("gap_ratio", 0.05);

    for (const auto& cj : j["chains"]) {
        Chain chain;
        chain.id = cj["id"];
        chain.name = cj["name"];
        auto col = cj["color"];
        for (int i = 0; i < 4; i++) chain.color[i] = col[i].get<float>();
        chain.head_direction = static_cast<SpawnFace>(cj["head_direction"].get<int>());

        for (const auto& bj : cj["blocks"]) {
            auto p = bj["pos"];
            IVec3 pos{p[0].get<int>(), p[1].get<int>(), p[2].get<int>()};
            chain.blocks.push_back(pos);

            // Reconstruct grid cell
            GridCell& cell = world.grid[pos];
            cell.pos = pos;

            // Determine which axis this chain uses at this position
            // For the first block, use head_direction's axis
            // For subsequent blocks, compute from consecutive positions
            int axis = -1;
            int block_idx = (int)chain.blocks.size() - 1;
            if (block_idx > 0) {
                IVec3 prev = chain.blocks[block_idx - 1];
                IVec3 diff = pos - prev;
                for (int ax = 0; ax < 3; ax++) {
                    if ((&diff.x)[ax] != 0) { axis = ax; break; }
                }
            } else {
                // First block: use head_direction axis
                int sign;
                FaceToAxisSign(chain.head_direction, axis, sign);
            }
            if (axis >= 0) cell.chain_on_axis[axis] = chain.id;

            // Turn info
            if (bj.value("is_turn", false)) {
                cell.is_turn = true;
                cell.turn_entry_axis = bj["turn_entry_axis"];
                cell.turn_exit_axis = bj["turn_exit_axis"];
            }
        }
        world.chains.push_back(chain);
    }

    if (!world.chains.empty()) {
        world.active_chain_id = 0;
    }
    return true;
}
```

---

## 11. Simulation Compilation Pipeline (Phase 2)

This section documents the Phase 2 compilation from `ChainWorld` to MuJoCo physics model.
**Implementation is deferred** but the architecture must be understood during Phase 1 to
ensure data structures support it.

### 11.1 Overview

```
ChainWorld  ──────►  mjSpec  ──────►  mjModel + mjData
(grid data)         (spec API)       (compiled physics)
```

### 11.2 CompileWorld() — Grid → mjSpec

Lives in `chainmaker_compile.cpp`:

```cpp
struct CompileResult {
    mjSpec* spec = nullptr;
    mjModel* model = nullptr;
    mjData*  data  = nullptr;
    std::string error;

    // Mapping: grid position → body name (for collision filter lookups)
    std::unordered_map<IVec3, std::string, IVec3Hash> pos_to_body;

    // Mapping: body index → (chain_id, chain_position_index)
    // Used by collision filter callback
    std::vector<std::pair<int, int>> body_chain_info;
};

CompileResult CompileWorld(const ChainWorld& world);
```

### 11.3 Compilation Steps

```
CompileWorld(world):
    1. spec = mj_makeSpec()

    2. Configure solver options (performance-tuned for chain dynamics):
       spec->option.jacobian   = mjJAC_SPARSE      // mandatory: nv = 3×nblocks grows fast
       spec->option.solver     = mjSOL_NEWTON       // quadratic convergence, fewer iters
       spec->option.integrator = mjINT_IMPLICITFAST
       spec->option.timestep   = 0.002
       spec->option.iterations = 20                 // 50→20: chains are well-conditioned
       spec->option.tolerance  = 1e-6               // 1e-8→1e-6: sufficient for structure

       // Per-block geom: condim=1 (frictionless contacts)
       // Chains are structural bones — friction adds solver DOF with no physical benefit.
       // Each contact: 1 constraint instead of 3-6, cutting solver cost ~3×.

    3. Add environment:
       world_body = mjs_findBody(spec, "world")
       - Floor geom (mjGEOM_PLANE, contype=1, conaffinity=7)
       - Directional lights (2x, same as progui)
       - Texture + material for checker floor (same as progui)

    4. Initialise three maps (all keyed by grid position IVec3):
         body_name_map     : IVec3 → string   (spec body name, first body at each cell)
         body_ptr_map      : IVec3 → mjsBody* (spec body pointer, first body at each cell)
         body_chain_id_map : IVec3 → int      (chain_id of the chain that first claimed the cell)
       These are used for (a) junction linking across chains,
       (b) differentiating self-intersection vs cross-chain intersection, and
       (c) post-compile chain-info lookup via mj_name2id.
       prev_body_name (std::string) is tracked alongside prev_body throughout the
       per-chain loop so that equality constraints can reference bodies by name.

    5. For each chain ci = 0..N-1  (see §11.7 for multi-chain details):

       a. Detect secondary chain:
            is_secondary = (ci > 0) && body_ptr_map.count(chain.blocks[0])
            if is_secondary && chain.Size() < 2 → skip

       b. Primary root (not is_secondary):
            root = mjs_addBody(world_body, NULL)
            root->pos = GridToWorld(chain.blocks[0])   // absolute world position
            Add freejoint (mjJNT_FREE, damping = kJointDamping)
            Add box geom, set chain colour
            body_name_map[blocks[0]]     = root name
            body_ptr_map[blocks[0]]      = root
            body_chain_id_map[blocks[0]] = ci
            prev_body = root;  prev_body_name = root name;  start_idx = 1

       c. Secondary root (is_secondary):
            junction = chain.blocks[0]                 // shared with earlier chain
            b1       = chain.blocks[1]                 // first EXTENSION block
            delta    = b1 - junction

            root = mjs_addBody(body_ptr_map[junction], NULL)
                                       //  ↑ parented UNDER junction body, not worldbody
            root->pos[0..2] = delta * CellStride()     // relative to junction parent
            Add ball joint (mjJNT_BALL) with anchor + cone limits (same formula as §11.4)
            Add box geom, set chain colour
            body_name_map[b1]     = root name  (if not already set)
            body_ptr_map[b1]      = root       (if not already set)
            body_chain_id_map[b1] = ci         (if not already set)
            prev_body = root;  prev_body_name = root name;  start_idx = 2

       d. Remaining blocks bi = start_idx..chain.Size()-1:
            delta     = chain.blocks[bi] - chain.blocks[bi-1]
            cur_pos   = chain.blocks[bi]

            cell_occupied        = body_ptr_map.count(cur_pos) > 0
            existing_name        = cell_occupied ? body_name_map[cur_pos] : ""
            is_self_intersection = cell_occupied &&
                                   body_chain_id_map[cur_pos] == ci   // same chain

            // ── Cross-chain through-junction ───────────────────────────────
            if cell_occupied && !is_self_intersection && existing_name not empty:
                // The current cell belongs to a DIFFERENT chain.
                // Add mjEQ_CONNECT (ball-and-socket) between prev_body and the
                // existing junction body — see §11.7.7 for full details.
                // Do NOT create a duplicate body (avoids visual overlap).
                AddConnectConstraint(spec, prev_body_name, existing_name, delta)
                prev_body      = body_ptr_map[cur_pos]
                prev_body_name = existing_name
                continue   // post-junction blocks attach as children of existing junc body

            // ── Normal block or self-intersection duplicate ─────────────────
            body  = mjs_addBody(prev_body, NULL)
            body->pos[0..2] = delta * CellStride()     // relative to parent
            Add box geom (condim=1), set chain colour
            Add ball joint with anchor + cone limits
            if !cell_occupied:                         // first body at this cell wins
                body_name_map[cur_pos]     = body name
                body_ptr_map[cur_pos]      = body
                body_chain_id_map[cur_pos] = ci
            prev_body      = body
            prev_body_name = body name

            // ── Self-intersection loop closure ──────────────────────────────
            if is_self_intersection && existing_name not empty:
                AddWeldConstraint(spec, existing_name, body name)
                // mjEQ_WELD rigidly couples two bodies at same grid position.
                // ONLY for same-chain loop closure — NOT for cross-chain.
                // Required: eq->objtype=mjOBJ_BODY, data[0..2]=0, data[3..5]=0,
                //           data[6]=1.0 (qw), data[10]=1.0 (torque ratio)

    6. Exclude close neighbours (grandparent–grandchild pairs) to avoid
       spurious collisions between directly-connected bodies (see §12).

    7. Compile:
       model = mj_compile(spec, NULL)
       if (!model) { error = mjs_getError(spec); return; }
       data = mj_makeData(model)
```

### 11.4 Ball Joint Cone Limit Formula

From progui reference (`progui_chain.cpp:599-616`):

```cpp
double ComputeConeLimit(double half_size, double gap) {
    double a = half_size;
    double g = gap;
    double c = 1.0 + (a > 0.0 ? (g / a) : 0.0);
    double root2 = std::sqrt(2.0);

    double phi;
    if (c >= root2) {
        phi = mjPI / 2.0;
    } else {
        phi = std::asin(std::max(0.0, std::min(1.0, c / root2))) - mjPI / 4.0;
    }

    // Clamp between 5° and 80°
    double th_min = mjPI * 5.0 / 180.0;
    double th_max = mjPI * 80.0 / 180.0;
    phi = std::max(th_min, std::min(th_max, phi));

    return phi * 180.0 / mjPI;  // convert to degrees
}
```

### 11.5 Simulation Loop

```cpp
// Hard cap: never run more than 16 mj_step() calls per display frame.
// This prevents the render loop from stalling when physics is slow.
static constexpr int kMaxStepsPerFrame = 16;

void StepSimulation(AppState& app) {
    auto wall_start = std::chrono::steady_clock::now();
    mjtNum simstart = app.sim_data->time;
    double target   = 1.0 / 60.0;
    int steps = 0;

    while ((app.sim_data->time - simstart) < target && steps < kMaxStepsPerFrame) {
        mj_step(app.sim_model, app.sim_data);
        ++steps;
    }

    // Update real-time ratio (EMA)
    double wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    if (wall_ms > 0.01) {
        double phys_ms = (app.sim_data->time - simstart) * 1000.0;
        g_rt_ratio = 0.9 * g_rt_ratio + 0.1 * (phys_ms / wall_ms);
    }
}
```

`g_rt_ratio` is a module-level EMA (exponential moving average) of physics_time / wall_time.
A value of `1.0` means real-time; values below `1.0` indicate the simulation is running slower than real-time.

### 11.6 Enter/Exit Simulation

```cpp
void EnterSimulation(AppState& app) {
    CompileResult result = CompileWorld(app.world);
    if (!result.model) {
        std::cerr << "Compile failed: " << result.error << "\n";
        return;
    }
    app.sim_spec  = result.spec;
    app.sim_model = result.model;
    app.sim_data  = result.data;
    app.mode = AppMode::SIMULATE;

    // Rebuild render context for the new model
    mjv_freeScene(&app.scn);
    mjr_freeContext(&app.con);
    mjv_makeScene(app.sim_model, &app.scn, 20000);
    mjr_makeContext(app.sim_model, &app.con, mjFONTSCALE_150);
}

void ExitSimulation(AppState& app) {
    if (app.sim_data) { mj_deleteData(app.sim_data); app.sim_data = nullptr; }
    if (app.sim_model) { mj_deleteModel(app.sim_model); app.sim_model = nullptr; }
    if (app.sim_spec) { mj_deleteSpec(app.sim_spec); app.sim_spec = nullptr; }

    // Rebuild render context for build mode (dummy model)
    mjv_freeScene(&app.scn);
    mjr_freeContext(&app.con);
    mjv_makeScene(NULL, &app.scn, 20000);
    mjr_makeContext(app.build_model, &app.con, mjFONTSCALE_150);
                 // ↑ use build_model (not NULL) — MuJoCo dereferences m unconditionally

    app.mode = AppMode::BUILD;
}
```

### 11.7 Multi-Chain Body Hierarchy

Multiple chains are combined into a single MuJoCo `mjSpec` / `mjModel`.  The
key design decision is **how secondary chains attach to the shared junction bead**.

#### 11.7.1 Two kinds of inter-chain interaction

There are two distinct ways a chain can interact with a block that belongs to a
different chain.  Each is handled differently in `CompileWorld`:

| Interaction | How it arises | Solution |
|---|---|---|
| **Secondary root** | User presses N on an existing block → `StartNewChainFromBlock` | Nested hierarchy — first extension body is a child of the junction body in the kinematic tree (§11.7.2) |
| **Cross-chain through-junction** | User presses Space while active block is adjacent to a different chain's block → `PlaceBlock` passes *through* the existing block | `mjEQ_CONNECT` ball-and-socket constraint (§11.7.7) |

**Why nested hierarchy for secondary roots (not `mjEQ_CONNECT`):**

An earlier attempt used `mjEQ_CONNECT` for secondary chain roots and failed:
- `mjsEquality.name1` / `name2` are resolved using `eq->objtype`.  The default
  value (`mjOBJ_UNKNOWN = 0`) causes MuJoCo's compiler to fall back to worldbody
  (empty name, id 0), producing:
  > `connect constraint supports only sites and bodies Element name '', id 0`
- Even with `objtype` set correctly, equality constraints add solver degrees of
  freedom and introduce constraint drift over time.

The **nested hierarchy** is simpler and physically accurate for secondary roots:
the first extension body becomes a direct **child** of the junction body in the
kinematic tree — no equality constraint needed.

#### 11.7.2 Body tree structure

```
worldbody
├── body_c0_0  [freejoint]          ← chain 0, block (0,0,0)
│   ├── body_c0_1  [ball joint]     ← chain 0, block (1,0,0)
│   │   └── body_c0_2  [ball joint] ← chain 0, block (2,0,0)
│   │       └── body_c0_3  ...
│   │
│   └── body_c1_1  [ball joint]     ← chain 1's first EXTENSION block (0,1,0)
│       └── body_c1_2  [ball joint] ← chain 1, block (0,2,0)
│           └── ...
│
└── body_c2_0  [freejoint]          ← chain 2 (starts at a different grid cell)
    └── ...
```

Rules:
- **Chain 0 (or any chain that starts at a fresh grid cell):** its root body is
  a child of `worldbody` with a `mjJNT_FREE` joint.  Its position is the
  absolute world-space coordinate of `blocks[0]`.
- **Secondary chain (starts at a cell already owned by an earlier chain):**
  `blocks[0]` is the **junction** — the shared bead.  `blocks[1]` is the first
  NEW bead belonging to this chain.  `body_c{ci}_1` is parented **under the
  junction body** (`body_ptr_map[blocks[0]]`) with a `mjJNT_BALL` joint, and
  its position is specified **relative to the junction body**.  It is physically
  as if the string of chain *ci* passes through the junction bead and continues
  outward.
- **Subsequent blocks** in all chains follow the standard nested pattern:
  each body is a child of the previous body with a ball joint, position relative
  to its parent.

#### 11.7.3 The "first body wins" rule and `body_chain_id_map`

Three maps are maintained, all applying a **first-body-wins** policy:

```cpp
if (!body_name_map.count(pos))     body_name_map[pos]     = name;
if (!body_ptr_map.count(pos))      body_ptr_map[pos]      = body;
if (!body_chain_id_map.count(pos)) body_chain_id_map[pos] = ci;
```

`body_chain_id_map` records **which chain_id first claimed each grid cell**.
This is the key that lets step 5d distinguish:
- **Cross-chain junction** (`body_chain_id_map[pos] != ci`): cell owned by a
  *different* chain → `mjEQ_CONNECT` + skip (§11.7.7)
- **Self-intersection** (`body_chain_id_map[pos] == ci`): same chain loops
  back to its own earlier position → `mjEQ_WELD` + duplicate body (§11.7.5)

Without `body_chain_id_map`, both cases would look identical (cell occupied) and
the old code applied WELD in both, creating a duplicate body at cross-chain
junctions that caused two overlapping geoms in simulation.

#### 11.7.4 Body count formula

For a world with N chains:

```
nbody = 1                                (worldbody)
      + Σ chain[0].Size()               (primary chain: all blocks get a body)
      + Σ (chain[i].Size() - 1)         (secondary root chains: junction block
        for secondary-root chains         is shared, only extensions get new bodies)
      - (cross-chain through-junctions)  (each PlaceBlock intersection that
                                          passes through another chain's block
                                          skips creating a body at that cell)
```

**Secondary-root chain** (via `StartNewChainFromBlock`): adds `chain.Size()-1` bodies
(junction body already exists; only extension blocks create new bodies).

**Cross-chain through-junction** (via `PlaceBlock` passing through another chain's block):
the junction position in the path is skipped (`continue`) — no duplicate body.
Each such junction reduces the body count by 1.

Example: chain 0 with 4 blocks + chain 1 with 7 path entries where 1 is a
cross-chain through-junction at chain 0's root:

```
nbody = 1             (worldbody)
      + 4             (chain 0: body_c0_0 .. body_c0_3)
      + (7 - 1 - 1)   (chain 1: starts secondary → -1 for junction block;
                        one through-junction skipped → -1 more)
      = 10   ← verified by test 9g
```

#### 11.7.5 Self-intersection within one chain (mjEQ_WELD)

When a **single chain's path** returns to a cell it already visited
(`body_chain_id_map[cur_pos] == ci`), the compile step creates a
**second body at the same world position** (child of the body just before the
revisit in path order). This second body:
- Has the same rest-pose coordinates as the original body.
- Is linked to the original body via **`mjEQ_WELD`** — added automatically
  in step 5d of `CompileWorld` (§11.3) when `is_self_intersection` is true.
- The weld rigidly couples the two bodies, closing the kinematic loop.

> **Important:** WELD is used **only for same-chain self-intersections**.
> Cross-chain through-junctions use `mjEQ_CONNECT` instead (§11.7.7)
> and do **not** create a duplicate body.

**WELD setup in spec API** (critical — default data is wrong for WELD):
```cpp
auto AddWeldConstraint = [&](const std::string& b1, const std::string& b2) {
    mjsEquality* eq = mjs_addEquality(spec, nullptr);
    eq->type    = mjEQ_WELD;
    eq->objtype = mjOBJ_BODY;
    mjs_setString(eq->name1, b1.c_str());
    mjs_setString(eq->name2, b2.c_str());
    // IMPORTANT: mjs_defaultEquality sets data[1]=1 (for CONNECT anchor).
    // Override ALL data fields explicitly for WELD:
    eq->data[0] = eq->data[1] = eq->data[2] = 0.0;   // anchor = origin
    eq->data[3] = eq->data[4] = eq->data[5] = 0.0;   // relpose pos = 0
    eq->data[6] = 1.0;   // qw = 1 → identity rotation
    eq->data[7] = eq->data[8] = eq->data[9] = 0.0;
    // data[10] = 1.0 (torque:force ratio) kept from mjs_defaultEquality
};
```

#### 11.7.6 Junction detection in CompileWorld

```cpp
bool is_secondary = (ci > 0) && body_ptr_map.count(chain.blocks[0]);
```

A chain is secondary iff:
1. It is not the first chain (`ci > 0`), **and**
2. Its first block (`blocks[0]`) already has a body in `body_ptr_map`
   (i.e. an earlier chain placed a body there via `StartNewChainFromBlock`).

Chains that start at an unoccupied grid cell always get a free joint, even
if `ci > 0` (e.g. two independent chains that share no junction).

Cross-chain through-junctions (§11.7.7) are detected *mid-path* during the
remaining blocks loop, using `body_chain_id_map`, and are independent of the
`is_secondary` flag.

#### 11.7.7 Cross-chain through-junction (mjEQ_CONNECT)

When `PlaceBlock` places a bead **through** an existing bead from a different
chain, both chains occupy the same grid cell simultaneously.  In the editor, one
chain's bead is the *junction bead* and the other chain's string passes through
it — like threading a string through a hole in a bead.

**The bug (pre-fix):** `CompileWorld` created a duplicate body at the junction
cell and added `mjEQ_WELD`.  This produced two overlapping box geoms in
simulation, making it look like the second chain's blocks were "inside" the
junction.

**The fix:** for cross-chain intersections, **no duplicate body is created**.
Instead:

1. An `mjEQ_CONNECT` (ball-and-socket, 3 positional DOF) constraint is added
   between `prev_body` (the approaching body in chain ci) and the existing
   junction body.  This is physically correct: the string is constrained to pass
   through the bead, but rotation is free.

2. The existing junction body becomes the new `prev_body` for chain ci, so
   all subsequent blocks in chain ci attach as **children of the junction body**
   in the kinematic tree.  No extra body at the junction position.

**`mjEQ_CONNECT` setup:**

```cpp
auto AddConnectConstraint = [&](const std::string& b1, const std::string& b2,
                                 const IVec3& delta) {
    mjsEquality* eq = mjs_addEquality(spec, nullptr);
    eq->type    = mjEQ_CONNECT;
    eq->objtype = mjOBJ_BODY;
    mjs_setString(eq->name1, b1.c_str());
    mjs_setString(eq->name2, b2.c_str());
    // Zero ALL data first: mjs_defaultEquality sets data[1]=1 (bad for CONNECT).
    for (int i = 0; i < 11; i++) eq->data[i] = 0.0;
    // Anchor1 in b1's local frame: face of b1 pointing toward b2
    // Anchor2 in b2's local frame: face of b2 pointing back toward b1
    // At rest pose both anchors coincide at the gap midpoint → zero residual.
    int axis = -1, sign = 1;
    for (int ax = 0; ax < 3; ax++) {
        if ((&delta.x)[ax] != 0) { axis = ax; sign = (&delta.x)[ax]; break; }
    }
    if (axis >= 0) {
        double anchor = half + gap * 0.5;   // midpoint of inter-bead gap
        eq->data[axis]     =  sign * anchor;  // anchor1
        eq->data[3 + axis] = -sign * anchor;  // anchor2
    }
};
```

**Anchor math (zero residual at rest):**

`b1` (approaching body) is at world position `P1`; the junction body (`b2`) is
at `P2 = P1 + delta * CellStride()`.  The gap between their faces is `gap`.

- `anchor1` in `b1` local frame = `+sign * (half + gap/2)` along the approach axis
- `anchor2` in `b2` local frame = `-sign * (half + gap/2)` along the same axis

In world coords at rest: `P1 + anchor1 = P1 + sign*(half+gap/2)` and
`P2 + anchor2 = P1 + CellStride - sign*(half+gap/2)`.
Since `CellStride = 2*half + gap`, both equal `P1 + sign*(half+gap/2)`. ✓

**Kinematic tree after through-junction:**

```
worldbody
└── body_c0_0  [free]
    └── ...
        └── body_c0_junc  [ball]   ← chain 0 owns the junction cell
            ├── body_c0_next [ball]   chain 0 continues
            └── body_c1_post [ball]   chain 1 continues AFTER junction

body_c1_root  [free]  (chain 1's independent root, parented to worldbody)
    └── ...
        └── body_c1_pre  [ball]
            + mjEQ_CONNECT: body_c1_pre ↔ body_c0_junc
```

**`prev_body_name` requirement:**

The `mjs_setString(eq->name1, ...)` call references bodies by name, not pointer.
`prev_body_name` (a `std::string`) must be maintained alongside `prev_body`
throughout the loop.  It is initialised to the root body name in both the primary
and secondary branches, and updated to `bname` whenever a new body is created.

## 12. Collision Filtering

### 12.1 Strategy (Three Layers)

1. **Parent-child auto-filter (free):** MuJoCo automatically disables collisions between
   parent and child bodies. The nested hierarchy gives this for free.

2. **Exclude pairs:** For grandparent-grandchild (2 steps apart):
   ```cpp
   for each chain:
       for i in 0..chain.Size()-3:
           mjsExclude* ex = mjs_addExclude(spec);
           mjs_setString(ex->bodyname1, body_name[i]);
           mjs_setString(ex->bodyname2, body_name[i+2]);
   ```

3. **Custom contact filter callback** (for chain-distance ≥ 3 filtering):
   ```cpp
   // Global lookup table: body_id → (chain_id, position_in_chain)
   static std::vector<std::pair<int,int>> g_body_chain_info;

   int ChainmakerContactFilter(const mjModel* m, mjData* d, int geom1, int geom2) {
       int body1 = m->geom_bodyid[geom1];
       int body2 = m->geom_bodyid[geom2];

       // World body always collides
       if (body1 == 0 || body2 == 0) return 0;

       auto [chain1, pos1] = g_body_chain_info[body1];
       auto [chain2, pos2] = g_body_chain_info[body2];

       // Different chains → always collide
       if (chain1 != chain2) return 0;

       // Same chain → filter if too close
       int distance = std::abs(pos1 - pos2);
       if (distance < 3) return 1;  // filter out (don't collide)

       return 0;  // allow collision
   }

   // Install before simulation:
   mjcb_contactfilter = ChainmakerContactFilter;
   ```

**WARNING:** Setting `mjcb_contactfilter` **replaces** the default `contype`/`conaffinity`
bitmask filter. The callback must replicate any bitmask logic needed (e.g., floor collision).
The callback above allows all cross-chain and floor collisions, which is correct.

---

## 13. Profiling & Performance Monitoring

### 13.1 Timer Callback

MuJoCo's profiling timers require a time callback returning **milliseconds** (not seconds).
The callback is installed in `EnterSimulation` before the first `mj_step`:

```cpp
static mjtNum ChainmakerTimer() {
    static auto t0 = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0);
    return static_cast<mjtNum>(elapsed.count());  // milliseconds
}

// Install before simulation:
mjcb_time = ChainmakerTimer;
```

**Critical:** `mjcb_time` must return milliseconds. `d->timer[i].duration` accumulates
values in the same units as the callback returns.

### 13.2 Reading Timer Data

```cpp
// Average milliseconds per step for timer slot i:
double avg_ms(const mjData* d, int slot) {
    if (d->timer[slot].number <= 0) return 0.0;
    return d->timer[slot].duration / d->timer[slot].number;
}

// Key timer slots:
//   mjTIMER_STEP           — total step (highest-level)
//   mjTIMER_POS_COLLISION  — broadphase + narrowphase collision
//   mjTIMER_CONSTRAINT     — constraint solver (Newton iterations)
//   mjTIMER_POS_KINEMATICS — forward kinematics
//   mjTIMER_POS_INERTIA    — composite inertia
```

### 13.3 Profiler Overlay

`RenderProfilerOverlay(const AppState& app, mjrRect viewport)` in `chainmaker_sim.cpp`:

- **Always-visible compact one-liner** (bottom-right): step ms, contact count, RT ratio, SLOW warning
- **P key toggles full breakdown** showing per-category milliseconds and percentages
- **`[REC]` tag** appears when video recording is active

```
--- Profiler -------------------
Step total :   2.45 ms
  Collision:   0.32 ms  ( 13%)
  Solver   :   1.85 ms  ( 76%)
  Kinematics:  0.12 ms  (  5%)
  Inertia  :   0.16 ms  (  7%)
Contacts   : 24
DOF (nv)   : 21    Bodies: 7
RT ratio   : 1.00x
Arena/Stack: 6291456 / 0
Recording  : OFF
--------------------------------
```

### 13.4 Real-Time Ratio

```cpp
static double g_rt_ratio = 0.0;  // module-level EMA

// Updated in StepSimulation() each frame:
g_rt_ratio = 0.9 * g_rt_ratio + 0.1 * (phys_ms / wall_ms);
```

- `1.0` = real-time  
- `< 1.0` = slower than real-time (SLOW warning shown)
- Step > 16ms triggers ` !! SLOW` warning (can't keep 60 Hz)

### 13.5 `get_profiler` IPC Command

Returns a snapshot of timer data for automated testing and monitoring:

```json
{"cmd": "get_profiler"}
```

Response:
```json
{
  "ok": true,
  "step_ms": 2.45,
  "collision_ms": 0.32,
  "solver_ms": 1.85,
  "kinematics_ms": 0.12,
  "inertia_ms": 0.16,
  "ncon": 24,
  "nbody": 7,
  "nv": 21,
  "iterations": 20
}
```

Only valid in `SIMULATE` mode. Returns `{"ok": false, "error": "not in simulation mode"}` otherwise.

### 13.6 Performance Optimizations Applied

| Parameter | Default | Optimized | Rationale |
|-----------|---------|-----------|-----------|
| `iterations` | 50 | **20** | Ball-joint chains are well-conditioned; Newton converges fast |
| `tolerance` | 1e-8 | **1e-6** | Structural simulation doesn't need sub-micron accuracy |
| `condim` (per geom) | 3 | **1** | Frictionless bead contacts; 1 constraint vs 3–6 per contact |
| Jacobian | `mjJAC_AUTO` | **`mjJAC_SPARSE`** | Mandatory for nv > 60 (3 DOF/block × N blocks) |
| Steps/frame | unlimited | **16 cap** | Prevents render loop stall when physics is slow |



## 14. Constants & Configuration

### 14.1 Physics Constants (from progui reference)

```cpp
// chainmaker.h or a dedicated constants section

// Joint damping — resists angular velocity at each ball joint
constexpr double kJointDamping = 0.15;

// Contact solver parameters
constexpr double kSolref[2] = {0.15, 0.7};    // timeconst, dampratio
constexpr double kSolimp[5] = {0.9, 0.95, 0.001, 0.5, 2.0};

// Geom margin for collision detection
constexpr double kGeomMargin = 0.0;

// Default bead size (edge length) — 5cm cube
constexpr double kDefaultBeadSize = 0.05;   // 5cm full edge
constexpr double kDefaultGapRatio = 0.05;   // 5% of bead_size

// Junction constraint stiffness
constexpr double kJunctionSolref[2] = {0.005, 1.0};  // very stiff

// Minimum collision distance along chain (blocks closer than this don't collide)
constexpr int kMinCollisionDistance = 3;

// Scene capacity
constexpr int kMaxSceneGeoms = 20000;

// UI panel width
constexpr int kUIWidth = 220;
```

### 14.2 SpawnFace Name Helper

```cpp
inline const char* FaceName(SpawnFace face) {
    static const char* names[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
    return names[static_cast<int>(face)];
}
```

### 14.3 Chain Name Helper

```cpp
inline std::string GenerateChainName(int id) {
    std::string name = "Chain ";
    name += (char)('A' + (id % 26));
    if (id >= 26) name += std::to_string(id / 26);
    return name;
}
```

---

## 15. Reference Patterns from progui/

### 15.1 Files to Study

| File | Key patterns to reuse |
|------|----------------------|
| `progui/main_progui.cc` | GLFW init, mjUI setup, render loop structure, mjuiDef array construction |
| `progui/progui_globals.h` | ChainEntry struct, SpawnFace enum, constants, global state pattern |
| `progui/progui_chain.cpp` | `spawnCube()` body/joint/geom creation, ball joint limits, gap computation |
| `progui/progui_ui.cpp` | Keyboard callback with direction keys, mouse callback with mjui_event, file IO capture |
| `progui/progui_sim.cpp` | `RebuildRenderContext()`, `FaceToAxisSign()`, gap adjustment |
| `progui/CMakeLists.txt` | Link targets, compile options, VS startup project |

### 15.2 Key Differences from progui

| Aspect | progui (V1) | ChainMaker (V2) |
|--------|-------------|-----------------|
| Build stage physics | `mj_compile()` + `mj_recompile()` per block | **None** — pure grid + `mjv_initGeom()` |
| Data model | `std::vector<ChainEntry>` (linear list of spec bodies) | `ChainWorld` with grid map + chain paths |
| Chains | Single chain only | Multiple chains with junction sharing |
| MuJoCo model during build | Always present (`mjModel* m`, `mjData* d`) | **NULL** — no model until simulation |
| Turn blocks | Implicit (direction change recorded in history) | Explicit (grid cell marked, entry/exit axes stored) |
| Save format | Direction tokens text file | JSON via nlohmann/json |
| Rendering during build | Full `mjv_updateScene(m, d, ...)` | Manual `mjv_initGeom()` per block |
| Z-axis keys | `5` / `6` | `Z` / `X` |
| Block selection | N/A (no junction support) | Ray-AABB picking for junction blocks |

### 15.3 What to Keep from progui

- **Ball joint creation pattern** — exact same joint setup with cone limits
- **Floor + lighting setup** — texture, material, directional lights
- **UI widget construction** — mjuiDef array with section, buttons, sliders
- **Camera interaction** — `mjv_moveCamera()` in mouse callbacks
- **FaceToAxisSign() helper** — exact same logic
- **Gap ratio concept** — `gap = bead_size * gap_ratio`

### 15.4 What NOT to Keep from progui

- **`mjSpec* spec` during build** — ChainMaker has no spec during build
- **`mj_recompile()` per block** — this is the performance bottleneck V2 eliminates
- **`g_lastBody` pointer tracking** — V2 uses grid positions, not body pointers
- **`ChainEntry` with spec body pointers** — V2 `Chain` uses `IVec3` positions
- **`LoopContactCheck()`** — V2 intersection is grid-based (check `grid.find(pos)`)
- **Direction history** — V2 stores explicit block positions, not relative directions

---

## 16. Implementation Checklist

All phases are **complete**. This checklist is preserved for historical reference.

### Phase 1A: Skeleton & Window ✅

- [x] Create `chainmaker/` directory
- [x] Create `chainmaker/CMakeLists.txt` (§2.2)
- [x] Add `CHAINMAKER` gate in root `CMakeLists.txt` (§2.1)
- [x] Create `chainmaker/chainmaker.h` with all data structures (§4)
- [x] Create `chainmaker/main.cc` — GLFW window, empty render loop, cleanup (§5)
- [x] Verify: window opens, gray background renders, no crash

### Phase 1B: Rendering ✅

- [x] Create `chainmaker/chainmaker_render.h/cpp` (§6)
- [x] Implement `PopulateBuildScene()` — floor only first
- [x] Implement `AddBlockGeom()` — render a single cube
- [x] Implement `AddGhostGeom()` — semi-transparent wireframe preview
- [x] Implement `AddHeadMarker()` — yellow sphere on head
- [x] Implement `AddDirectionArrow()` — arrow pointing in spawn direction
- [x] Implement `ComputeBlockColor()` — chain-based coloring
- [x] Verify: single block + ghost + arrow visible in viewport

### Phase 1C: Chain Logic ✅

- [x] Create `chainmaker/chainmaker.cpp` (§9)
- [x] Implement `CreateFirstChain()` — one block at origin
- [x] Implement `PlaceBlock()` — extend chain with validation
- [x] Implement `DeleteLastBlock()` — remove last block
- [x] Implement `SetDirection()` — change spawn face with reversal guard
- [x] Implement `SwitchChain()` — switch active chain by index
- [x] Implement turn block detection (mark head as turn when direction changes)
- [x] Implement intersection detection (chain threads through existing straight block)
- [x] Verify: can place 20+ blocks, change direction, see turns marked

### Phase 1D: UI & Input ✅

- [x] Create `chainmaker/chainmaker_ui.h/cpp` (§7, §8)
- [x] Implement `BuildUI()` — mjUI panel with buttons and sliders
- [x] Implement `KeyboardCallback()` — all keybinds (§8.1)
- [x] Implement `MouseButtonCallback()` — UI clicks + camera (§8.2)
- [x] Implement `MouseMoveCallback()` — camera orbit/pan (§8.3)
- [x] Implement `ScrollCallback()` — zoom (§8.4)
- [x] Implement status bar overlay (§7.5)
- [x] Verify: all keyboard shortcuts work, UI buttons trigger actions

### Phase 1E: Multi-Chain ✅

- [x] Implement `StartNewChainFromBlock()` — junction creation (§9.6)
- [x] Implement `RayPickBlock()` — click-to-select block (§9.7)
- [x] Implement `NEW_CHAIN_PICK` mode flow (click block → pick direction → new chain)
- [x] Implement chain switching (1-9 keys)
- [x] Implement junction rendering (blended colors)
- [x] Implement validation: turn blocks can't be junctions
- [x] Verify: can create 3+ chains sharing junction blocks

### Phase 1F: Save/Load ✅

- [x] Create `chainmaker/chainmaker_io.h/cpp` (§10)
- [x] Implement `SaveWorldToJSON()` — serialize world to JSON file
- [x] Implement `LoadWorldFromJSON()` — deserialize JSON to world
- [x] Implement file I/O key capture mode (§8.5)
- [x] Verify: save → close → reopen → load produces identical structure

### Phase 2A: Simulation Compilation ✅

- [x] Create `chainmaker/chainmaker_compile.h/cpp` (§11)
- [x] Implement `CompileWorld()` — grid → mjSpec → mjModel
- [x] Implement nested body hierarchy builder
- [x] Implement ball joint creation with cone limits
- [x] Implement `mjEQ_WELD` for self-intersecting loop closure (§11.7.5)
- [x] Implement collision filtering (§12)
- [x] Verify: compiled model doesn't crash, blocks stay connected

### Phase 2B: Simulation Runtime ✅

- [x] Create `chainmaker/chainmaker_sim.h/cpp` (§13)
- [x] Implement `EnterSimulation()` / `ExitSimulation()`
- [x] Implement simulation loop with step cap (kMaxStepsPerFrame=16)
- [x] Install profiling timer (returns ms) and callback
- [x] Implement compact + full profiler overlay (P key toggle)
- [x] Implement real-time ratio EMA tracking
- [x] Implement `get_profiler` IPC command
- [x] Implement video recording via ffmpeg pipe (F9 + IPC, §18)
- [x] Verify: simulation runs, blocks swing under gravity, can return to build
- [x] 95/95 automated tests passing

---

## Appendix A: MuJoCo API Quick Reference

### Visualization (Build Stage)

| Function | Purpose | Key Parameters |
|----------|---------|----------------|
| `mjv_makeScene(NULL, &scn, maxgeom)` | Allocate scene | model=NULL ok, maxgeom=20000 |
| `mjv_initGeom(g, type, size, pos, mat, rgba)` | Init one visual geom | All nullable except geom ptr |
| `mjr_makeContext(NULL, &con, fontscale)` | Create render context | model=NULL ok |
| `mjr_render(viewport, &scn, &con)` | Render scene | Standard call |
| `mjr_overlay(font, pos, vp, text, NULL, &con)` | Text overlay | Second text param unused |
| `mjv_moveCamera(NULL, action, dx, dy, &scn, &cam)` | Camera control | model=NULL ok |

### UI Widgets

| Type | mjuiDef.type | Usage |
|------|-------------|-------|
| Section header | `mjITEM_SECTION` | Groups controls, state=1 for open |
| Button | `mjITEM_BUTTON` | Click action, state=2 for enabled |
| Slider | `mjITEM_SLIDERNUM` | pdata=&variable, other="min max" |
| Static text | `mjITEM_STATIC` | Read-only display |
| Separator | `mjITEM_SEPARATOR` | Visual divider |
| End marker | `mjITEM_END` | Must be last entry |

### Spec API (Phase 2)

| Function | Returns | Purpose |
|----------|---------|---------|
| `mj_makeSpec()` | `mjSpec*` | Create empty spec |
| `mjs_findBody(spec, "world")` | `mjsBody*` | Get worldbody |
| `mjs_addBody(parent, NULL)` | `mjsBody*` | Add child body |
| `mjs_addGeom(body, NULL)` | `mjsGeom*` | Add geom to body |
| `mjs_addJoint(body, NULL)` | `mjsJoint*` | Add joint to body |
| `mjs_addEquality(spec, NULL)` | `mjsEquality*` | Add equality constraint |
| `mjs_addExclude(spec)` | `mjsExclude*` | Add collision exclude pair |
| `mjs_setName(element, name)` | void | Set element name |
| `mjs_setString(field, value)` | void | Set string field |
| `mj_compile(spec, NULL)` | `mjModel*` | Compile spec to model |
| `mj_makeData(model)` | `mjData*` | Create simulation data |
| `mj_deleteSpec(spec)` | void | Free spec |

### Geom Types

| Constant | Value | Usage |
|----------|-------|-------|
| `mjGEOM_BOX` | 6 | Solid cube (block) |
| `mjGEOM_LINEBOX` | — | Wireframe cube (ghost) |
| `mjGEOM_PLANE` | 0 | Floor |
| `mjGEOM_SPHERE` | 2 | Head marker |
| `mjGEOM_ARROW` | — | Direction indicator |

---

## Appendix B: Error Handling

### Build Stage Errors

| Condition | Response |
|-----------|----------|
| `scn.ngeom >= scn.maxgeom` | Skip geom, print warning once |
| Place into occupied incompatible cell | Reject silently, update status bar |
| Turn on junction block | Reject, show "Cannot turn on junction" |
| No available axis at junction | Reject, show "No axes available" |
| File save/load failure | Print to stderr, show in status bar |

### Simulation Errors (Phase 2)

| Condition | Response |
|-----------|----------|
| `mj_compile()` returns NULL | Show `mjs_getError(spec)`, stay in build mode |
| `mjWARN_CONTACTFULL` | Show warning, suggest increasing nconmax |
| `mjWARN_CNSTRFULL` | Show warning, suggest increasing njmax |
| Step time > 16ms | Show real-time warning in profiler overlay |

---

## Appendix C: Coordinate System Conventions

- MuJoCo uses **right-handed** coordinates: +X right, +Y forward, +Z up
- Grid origin `(0, 0, 0)` maps to world position `(0, 0, bead_size)` (lifted off floor)
- Grid coordinates are integers; physical positions are `grid * cell_stride`
- `cell_stride = bead_size * (1 + gap_ratio)`
- Camera initial position: lookat=(0,0,0.5), azimuth=135°, elevation=-30°, distance=3.0

---

## 17. IPC Test Server

ChainMaker supports a **TCP IPC test mode** (`--test` flag) that allows fully
automated, programmatic GUI testing without keyboard injection or window-focus
tricks. This makes it possible to run a Python test suite that exercises every
GUI feature without any risk of stray keystrokes landing in the wrong window.

### 17.1 Motivation

Testing a GLFW/OpenGL application via `pyautogui` keyboard injection is fragile:
- If ChainMaker loses focus (e.g., after closing a dialog), simulated keystrokes
  go to whichever window now has focus.
- On Windows this can inadvertently trigger file dialogs, switch sessions, or
  corrupt the test environment.

The IPC approach eliminates all of this. Every test interaction goes through a
local TCP socket — focus never changes hands.

### 17.2 Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│  ChainMaker process (main thread = GL thread)                    │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Main render loop (60 Hz)                                │   │
│  │    mj_step / mjv_updateScene / mjr_render                │   │
│  │    IpcProcessCommands(app, viewport)  ← executes 1 cmd   │   │
│  └──────────────────────────────────────────────────────────┘   │
│           ▲ fulfills promise / pops queue                        │
│           │                                                      │
│  ┌────────┴─────────────────────────────────────────────────┐   │
│  │  IPC background thread                                   │   │
│  │    accept() one client at a time                         │   │
│  │    recv() newline-delimited JSON command                 │   │
│  │    push IpcCommand{json_in, promise} to queue            │   │
│  │    future.get()  ← blocks until main thread responds     │   │
│  │    send() newline-delimited JSON response                │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
         ▲ localhost:47832 TCP
         │
┌────────┴──────────────────────────────┐
│  Python test script                   │
│    ChainMakerClient.send(cmd) → dict  │
└───────────────────────────────────────┘
```

**Key design choices:**

| Decision | Reason |
|----------|--------|
| All commands execute on the main GL thread | OpenGL is single-threaded; `mjr_readPixels` and all render calls must run there |
| One command per frame | Keeps the render loop responsive; 60fps → ≤16ms latency |
| `std::promise/future` | Clean synchronisation — background thread blocks until main thread has a result; no polling |
| Newline-delimited JSON | Dead-simple framing; works with Python's `socket.recv` buffering model |
| localhost only (127.0.0.1) | Security: no remote access |

### 17.3 Files

| File | Purpose |
|------|---------|
| `chainmaker/chainmaker_ipc.h` | Public API: `IpcServerStart`, `IpcServerStop`, `IpcProcessCommands` |
| `chainmaker/chainmaker_ipc.cpp` | Full implementation: WinSock2 threading, command dispatch, screenshot |
| `chainmaker/test_chainmaker_client.py` | Python client library: `ChainMakerClient` class |
| `chainmaker/test_chainmaker.py` | Python test suite: 95 tests across all GUI features |

### 17.4 Activation

Launch ChainMaker with the `--test` flag:

```
ChainMaker.exe --test
```

This sets `app.test_mode = true`, calls `IpcServerStart(app)`, and begins the
background accept thread. If `--test` is not passed, the IPC server is never
started and has zero overhead.

### 17.5 Threading Model

```cpp
// IpcServer internal structure (chainmaker_ipc.cpp)
struct IpcServer {
    std::thread         thread;           // background accept/recv thread
    std::atomic<bool>   running{true};
    std::mutex          mutex;
    std::queue<std::shared_ptr<IpcCommand>> queue;
    SOCKET              server_sock;
    SOCKET              client_sock;
};

struct IpcCommand {
    std::string             json_in;
    std::promise<std::string> promise;
};
```

**Background thread flow:**
1. `accept()` with 200ms `select()` timeout — allows clean shutdown check
2. On connection: loop `recv()` until newline; push `IpcCommand` to queue
3. `future.get()` — blocks until main thread processes it
4. `send(response + "\n")` back to Python client
5. Repeat for next command on same connection; on disconnect, return to `accept()`

**Main thread flow (once per frame, after `mjr_render`):**
```cpp
void IpcProcessCommands(AppState& app, mjrRect viewport) {
    if (!app.ipc) return;
    std::shared_ptr<IpcCommand> cmd;
    { /* lock */ if (queue.empty()) return; cmd = queue.front(); queue.pop(); }
    json response = ExecuteCommand(json::parse(cmd->json_in), app, viewport);
    cmd->promise.set_value(response.dump());
}
```

### 17.6 Wire Protocol

All messages are UTF-8 JSON objects terminated by `\n`.

**Request format:**
```json
{"cmd": "<command_name>", [optional parameters]}
```

**Response format (success):**
```json
{"ok": true, [optional result fields]}
```

**Response format (failure):**
```json
{"ok": false, "error": "<human-readable message>"}
```

### 17.7 Command Reference

| Command | Parameters | Response fields | Description |
|---------|-----------|----------------|-------------|
| `get_state` | — | `mode`, `nchains`, `active_chain`, `nblocks`, `ngrid`, `head {x,y,z}`, `direction`, `bead_size`, `gap_ratio` | Full application snapshot |
| `place_block` | — | `nblocks`, `result` | Place block at current head |
| `delete_block` | — | `nblocks` | Delete last block in active chain |
| `set_direction` | `dir`: string | — | Set placement direction. Valid values: `+X`, `-X`, `+Y`, `-Y`, `+Z`, `-Z` |
| `new_chain_mode` | — | — | Enter NEW_CHAIN_PICK mode |
| `start_chain_at` | `x`, `y`, `z`: int | `chain_id` | Start new chain branching from grid cell (x,y,z) |
| `cancel_pick` | — | — | Cancel NEW_CHAIN_PICK → return to BUILD |
| `switch_chain` | `id`: int | — | Make chain `id` the active chain |
| `adjust_gap` | `delta`: float | `gap_ratio` | Adjust gap ratio by delta |
| `set_bead_size` | `size`: float | — | Set bead edge length (0.01–0.15 m) |
| `screenshot` | `path`: string | `width`, `height`, `path` | Capture current frame to PPM file |
| `enter_simulate` | — | `nbody`, `nv` | Compile world and enter physics |
| `exit_simulate` | — | — | Return to build stage |
| `get_profiler` | — | `step_ms`, `collision_ms`, `solver_ms`, `kinematics_ms`, `inertia_ms`, `ncon`, `nbody`, `nv`, `iterations` | Per-step timer data (SIMULATE mode only) |
| `start_recording` | `path` (optional): string | `path` | Start video recording to MP4 via ffmpeg. Auto-generates timestamped name if path omitted. |
| `stop_recording` | — | `was_recording` | Stop active recording and finalize MP4 |
| `get_recording_status` | — | `is_recording`, `path`, `width`, `height` | Recording state snapshot |
| `save` | `path`: string | — | Save world to JSON |
| `load` | `path`: string | — | Load world from JSON |
| `reset` | — | — | Clear world to single-block state |
| `quit` | — | — | Close ChainMaker window and exit |

### 17.8 Screenshot Implementation

Screenshots use `mjr_readPixels()` on the current render framebuffer, then write
a binary PPM (P6) file. OpenGL framebuffers are bottom-to-top; the writer
reverses row order to produce a standard top-to-bottom image.

```cpp
static void WriteScreenshot(const std::string& path, mjrRect vp,
                             mjrContext* con) {
    int W = vp.width, H = vp.height;
    std::vector<unsigned char> rgb(W * H * 3);
    mjr_readPixels(rgb.data(), nullptr, vp, con);

    FILE* f = fopen(path.c_str(), "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    // GL is bottom-to-top; PPM is top-to-bottom
    for (int row = H - 1; row >= 0; --row)
        fwrite(rgb.data() + row * W * 3, 1, W * 3, f);
    fclose(f);
}
```

Python reads PPM with `PIL.Image.open(path)` — no conversion needed.

### 17.9 Python Client API

```python
from test_chainmaker_client import ChainMakerClient, launch_chainmaker

# Option A: connect to a running instance
with ChainMakerClient() as c:
    c.place_block()
    c.set_direction("+Y")
    state = c.get_state()       # returns dict
    c.screenshot("frame.ppm")
    c.quit()

# Option B: launch ChainMaker.exe and connect automatically
with launch_chainmaker(exe_path="...", wait=3.0) as c:
    # same API
    pass
```

`ChainMakerClient` wraps every command as a typed Python method. All methods
return a `dict` parsed from the server's JSON response.

### 17.10 Running the Test Suite

```powershell
# Kill any stale instance
$p = Get-Process -Name ChainMaker -EA 0; if ($p) { Stop-Process -Id $p.Id -Force }

# Build
cmake --build build --config Release --target ChainMaker

# Run tests (launches ChainMaker internally, then kills it)
python chainmaker/test_chainmaker.py
```

Expected output on a clean build: **95 tests, 0 failures.**

Screenshots from each test phase are written to
`chainmaker/test_screenshots/*.ppm` for visual verification.

### 17.11 Extending the Test Suite

To add a new IPC command:

1. **C++ (`chainmaker_ipc.cpp`):** Add a branch in `ExecuteCommand()`:
   ```cpp
   if (cmd == "my_command") {
       // validate params, call app functions, build response
       res["ok"] = true;
       res["my_field"] = some_value;
       return res;
   }
   ```

2. **Python client (`test_chainmaker_client.py`):** Add a method:
   ```python
   def my_command(self, param: int) -> dict:
       return self._send({"cmd": "my_command", "param": param})
   ```

3. **Test suite (`test_chainmaker.py`):** Add a section with `ok(...)` assertions.

### 17.12 Security Notes

- The IPC server binds to `127.0.0.1` only — never accessible from the network.
- It is activated **only** with the `--test` flag — production use has zero attack surface.
- No authentication is needed since it is localhost-only and test-mode-only.
- The server accepts **one client connection at a time** — no concurrent access issues.

---

## 18. Video Recording

ChainMaker can record simulation (or build stage) output to MP4 video via ffmpeg.

### 18.1 Architecture

```
Render loop (each frame, after all rendering, before glfwSwapBuffers):
    if (app.is_recording):
        mjr_readPixels(rgb_buf, nullptr, viewport, &con)  // read back buffer
        fwrite(rgb_buf → ffmpeg stdin pipe)

ffmpeg process (spawned via popen):
    -f rawvideo -pixel_format rgb24 -video_size WxH -framerate 60 -i pipe:0
    -vf vflip    ← OpenGL is bottom-to-top; vflip corrects to top-to-bottom
    -c:v libx264 -preset fast -crf 22
    output.mp4
```

**Why `-vf vflip`:** `glReadPixels` (called by `mjr_readPixels`) returns rows in
bottom-to-top order (OpenGL convention). Video files expect top-to-bottom order.

### 18.2 AppState Recording Fields

```cpp
bool            is_recording    = false;
char            record_path[512] = "";
FILE*           record_pipe     = nullptr;   // popen handle
int             record_width    = 0;         // captured at StartRecording
int             record_height   = 0;
unsigned char*  record_pixels   = nullptr;   // width×height×3 RGB buffer
```

### 18.3 API

```cpp
// chainmaker_sim.h
bool StartRecording(AppState& app, const char* path);  // opens ffmpeg pipe
void StopRecording(AppState& app);                     // closes pipe, frees buffer
void ToggleRecording(AppState& app);                   // F9 key handler
void CaptureRecordingFrame(AppState& app, mjrRect viewport);  // call each frame
```

### 18.4 Activation

| Method | Behaviour |
|--------|-----------|
| **F9 key** (sim mode) | Toggle recording; auto-generates `chainmaker_YYYYMMDD_HHMMSS.mp4` |
| **IPC `start_recording`** | `{"cmd": "start_recording", "path": "out.mp4"}` |
| **IPC `stop_recording`** | `{"cmd": "stop_recording"}` |
| **IPC `get_recording_status`** | Returns `{"is_recording": bool, "path": "...", "width": W, "height": H}` |

### 18.5 Requirements

- **ffmpeg must be installed and in PATH.** `StartRecording` calls `popen(ffmpeg ...)`;
  if ffmpeg is not found, `popen` returns non-null (cmd.exe starts) but the pipe
  immediately breaks. The first `fwrite` call fails, `StopRecording` is called
  automatically, and `is_recording` becomes `false`.
- Recording works in **both BUILD and SIMULATE modes.**
- If the window is **resized** during recording, `CaptureRecordingFrame` detects
  the size mismatch and calls `StopRecording` gracefully.
- The profiler overlay shows `[REC]` when recording is active.

### 18.6 Cross-Platform Notes

Windows uses `_popen`/`_pclose`; POSIX uses `popen`/`pclose`. The file
`chainmaker_sim.cpp` maps these with:
```cpp
#ifdef _WIN32
#  define popen  _popen
#  define pclose _pclose
#endif
```

---

## 19. Scope Status

| Phase | Status | Description |
|-------|--------|-------------|
| **Phase 1** | ✅ Complete | Multi-chain GUI: placement, turns, junctions, save/load |
| **Phase 2** | ✅ Complete | Simulation: compile, physics loop, profiler, video recording, weld constraints |
| **Phase 3** | 🔲 Planned | Actuators: linear motors, solenoids, muscle-like tendons |
| **Phase 4** | 🔲 Planned | Sensors: force/torque, current, voltage monitoring |
| **Phase 5** | 🔲 Planned | Multi-robot: combine/coordinate multiple robot instances |
| **Phase 6** | 🔲 Planned | Automation: scripted movement playback |
| **Phase 7** | 🔲 Planned | Manufacturing export: BOM, standardized part IDs |

---

## 20. Known Requirements & Invariants

This section captures correctness requirements that are easy to accidentally violate.

### 20.1 Junction–Overlap Invariant

**Requirement:** When a new chain is created at an existing block (junction), the new chain's
first block is the junction block itself. PlaceBlock then walks forward in the chain's direction.
If the next cell in that direction is already occupied (another junction), the chain must register
on that cell's axis and continue walking — **not** place a second block on top of an already-occupied cell.

**Why it matters:** Without this walk, the second chain would have its head sitting visually
inside another block. The user sees "blocks inside blocks" which is wrong. Every block in the
`chain->blocks` list must map 1:1 to a unique position along the chain's axis of travel.

**Enforcement:** `PlaceBlock` iterates forward cell-by-cell until it finds a free cell:
```
while grid[cursor + dir] is occupied and axis is free:
    register chain on that cell
    walk cursor forward
land on the first free cell
```
`DeleteLastBlock` must undo the **entire walk** atomically using `chain->undo_stack`.

### 20.2 Atomic Undo Invariant

**Requirement:** Pressing Delete once always undoes exactly one `PlaceBlock()` call, no matter
how many junction cells were registered during that call.

**Enforcement:** `PlaceBlock` records `junctions.size() + 1` onto `chain->undo_stack`. 
`DeleteLastBlock` pops this count and removes all blocks in a single press.
After save/load `undo_stack` is empty — Delete then falls back to 1-block-per-press.

### 20.3 UI Click-Through Prevention

**Requirement:** Mouse clicks and scroll events on the UI panel must never propagate to the
3D world (camera rotation, ray-pick, etc.).

**Enforcement:** `mouserect = mjr_findRect(...)` is computed in every mouse callback.
If `mouserect == ui.rectid`, the event is consumed by `mjui_event` or silently ignored.
Camera movement is blocked. `dragrect` keeps slider events routing to the UI even if the
cursor drifts outside the panel during a drag.

### 20.4 Multi-Chain Color Integrity

**Requirement:** A junction block (shared by 2+ chains) renders with a blended color.
No chain should render its blocks as if they are the only chain at that cell.

**Enforcement:** During `PopulateScene`, each grid cell's chain list is inspected. If
`cell.ChainCount() > 1`, the color is the average of all chain colors at that cell.

### 20.5 `uistate.button` Must Be Set

**Requirement:** `mjui_event` returns NULL immediately if `uistate.button != mjBUTTON_LEFT`.
This means all UI buttons and sliders appear to do nothing unless `uistate.button` is
explicitly set to the correct `mjtButton` enum before every `mjui_event` call.

**Enforcement:** `MouseButtonCallback` translates GLFW button code → `mjtButton` before
calling `mjui_event`. See §8.2 and §8.6 for the exact translation.
