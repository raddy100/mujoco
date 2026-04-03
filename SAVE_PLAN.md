# ChainMaker — Save Plan (Session Checkpoint)

## Current Status: ALL 26 TASKS CODED, VIEWPORT BUG FIXED

### What Was Built
A new `chainmaker/` directory with a complete two-stage GUI for designing bead-on-string robot
structures. The app builds, launches, and the 3D viewport renders correctly.

### Files Created
```
chainmaker/
  chainmaker.h              — Core data structures, constants, all forward declarations
  chainmaker.cpp            — CreateFirstChain, PlaceBlock, DeleteLastBlock, SetDirection, etc.
  chainmaker_render.h/.cpp  — Build stage scene population (lights, floor, blocks, ghost, arrow)
  chainmaker_render_overlay.cpp — Status bar and mode overlays (part of chainmaker_render.cpp)
  chainmaker_ui.h/.cpp      — mjUI panel (buttons, sliders), all GLFW callbacks
  chainmaker_io.h/.cpp      — JSON save/load via nlohmann/json, file I/O key capture
  chainmaker_compile.h/.cpp — CompileWorld: grid → mjSpec → mjModel (Phase 2)
  chainmaker_sim.h/.cpp     — EnterSimulation, ExitSimulation, StepSimulation, profiler
  main.cc                   — GLFW window, render loop, UpdateBuildCamera()
  CMakeLists.txt            — FetchContent nlohmann/json, links mujoco+glfw+threads
```

Root CMakeLists.txt modified to include `add_subdirectory(chainmaker)` gated by `-DCHAINMAKER=ON`.

### Build Command
```
cmake -B build -DCHAINMAKER=ON
cmake --build build --config Release --target ChainMaker
```

### Key Bug Fixes Applied This Session
1. **mjSpec field names**: `spec->option.*` (NOT `spec->opt.*`); `j->damping[0]` (array, not scalar)
2. **Frustum crash**: `mjr_render` errors if `scn.camera[0/1].frustum_near < mjMINVAL`. Fixed by
   `UpdateBuildCamera()` in main.cc which manually computes GL camera pos/forward/up/frustum
   from `mjvCamera` azimuth/elevation/distance (since `mjv_updateCamera` requires a valid mjModel).
3. **Black viewport (ROOT CAUSE)**: `mjr_makeContext(nullptr, ...)` skips `makeBuiltin()` — the
   function that uploads box/sphere/arrow geometry VBOs to the GPU. Fixed by creating a minimal
   dummy `mjModel` (empty mjSpec → mj_compile) and passing it to `mjr_makeContext`. The dummy
   model is stored in `app.build_model` and freed at exit.
4. **Keyboard event routing**: Added `uistate.type = mjEVENT_KEY` + modifier fields in
   `KeyboardCallback` before calling `mjui_event`.

### What Remains To Be Tested
The app runs and the viewport renders, but the following need user verification:
- [ ] Blocks appear in correct colors when placed (SPACE/C keys)
- [ ] Direction changes (arrow keys, Z/X) work visually
- [ ] Ghost preview shows next block position
- [ ] Delete (DEL/BACKSPACE) removes last block
- [ ] Camera drag/scroll works
- [ ] New Chain (N) → click block → starts second chain
- [ ] Chain switch (1-9 keys) works
- [ ] Save (Ctrl+S) / Load (Ctrl+L) create/read JSON files
- [ ] Simulate (P) compiles model and enters physics
- [ ] Return to build (R) restores build stage

### GUI Testing Problem
**The Problem**: Using pyautogui for automated GUI testing backfires when ChainMaker loses focus or
the test triggers save/load filename prompts — keystrokes fall into the Copilot terminal instead,
causing session switches and test interruption.

### Proposed Testing Solutions (see below for discussion)
Three approaches are under consideration:
1. **IPC Test Server** (best): ChainMaker opens a local TCP socket in `--test` mode; test script
   sends JSON commands and reads JSON responses. No global key injection.
2. **Batch Script Mode**: `--run-script script.txt` flag; ChainMaker reads actions from a file and
   writes results to an output file. No GUI interaction.
3. **stdout Event Log**: ChainMaker prints structured JSON lines to stdout for every state change;
   test script launches ChainMaker as a subprocess and reads stdout.

### Architecture Notes
- Build stage: `mjModel* = NULL`. Geoms added manually via `mjv_initGeom` each frame.
- `mjv_makeScene(nullptr, ...)` — OK (allocates geom buffer only, no model dependency)
- `mjr_makeContext(nullptr, ...)` — NOT OK (skips VBO uploads); use dummy model instead
- `scn.camera[0]` and `scn.camera[1]` BOTH must have `frustum_near > mjMINVAL`
- Ball joints (NOT tendons) simulate string; nested body hierarchy = hard kinematic constraint
- Secondary chains connect via `mjEQ_CONNECT` equality constraint to junction body

### Phase 2 (Not Yet Tested)
CompileWorld and simulation runtime are coded but untested because the GUI test phase was
interrupted. The compilation path is:
`ChainWorld grid → mjSpec (nested bodies + ball joints + mjEQ_CONNECT) → mj_compile → mjModel`

---
*Saved: 2026-04-02 — Session 18c8180a*
