# ProGUI Refactor Design Overview

## 1. Original Legacy Structure

Legacy code is split across four C-style translation units using extensive global state:

- progui_globals.h / .cpp: Declares and defines all global variables (MuJoCo model handles, UI state, chain data structures, gap controls, button label strings, direction history, etc.) plus helper functions for direction recording (RecordTurnInput(), RecordForwardInput(), ClearDirectionHistory()).
- progui_chain.cpp: Implements chain construction, deletion, movement, loop formation, save/load, gap persistence, and direction replay logic with numerous internal `static` helper functions operating on globals.
- progui_sim.cpp: Houses rendering-context rebuild logic, physics option toggles, gap adjustment routines, probe contact queries, and mapping of spawn face to axis/sign.
- progui_ui.cpp: Implements GLFW input callbacks (keyboard, mouse buttons, mouse motion, scroll) performing direct mutation of global state and invoking free functions from other modules.
- main_progui.cc (not yet analyzed/refactored): Presumably creates window, initializes MuJoCo, sets callbacks, enters simulation loop.

### Key Characteristics

- Heavy reliance on mutable global variables.
- Free functions with implicit dependencies (must have `m`, `d`, `spec` initialized).
- Manual resource lifetime management scattered, no RAII wrappers.
- Many operations return `void` or `bool` without structured error reporting.
- Thread-safety is absent (all state assumed single-threaded access in main loop).
- Macros minimal; constants provided as global `const double` or `constexpr`-like variables.

## 2. Inventory of Global State (to be Privatized)

From progui_globals.cpp:

- Simulation handles: `mjModel* m`, `mjData* d`, `mjSpec* spec`
- Visualization: `mjvCamera cam`, `mjvOption opt`, `mjvScene scn`, `mjrContext con`
- UI: `mjuiState uistate`, `mjUI ui`
- Mouse/input: `bool button_left`, `button_middle`, `button_right`, `double lastx`, `lasty`
- Modes/flags: `bool g_boxesCollide`, `bool g_physicsEnabled`, `mjtNum g_savedGravity[3]`
- Chain tracking: `mjsBody* g_lastBody`, `std::vector<ChainEntry> g_chain`, `std::unordered_map<int,size_t> g_bodyIdToIndex`
- Markers / probe: `mjsGeom* g_lastMarker`, `mjsGeom* g_probeRect`, `std::string g_probeName`
- Gap: `double g_gapRatio`, `char g_gapRatioLabel[64]`
- IO: `char g_ioFilename[256]`, `int g_fileIoMode`
- Pre-physics save: `bool g_hasSavedPrePhysicsState`, `std::vector<SavedBodyPos> g_savedPrePhysicsChain`
- Constants: `kJointDamping`, `kGeomMargin`, `kSolref[2]`, `kSolimp[5]`, `kBoxHalf`, `g_uiWidth`
- Spawn orientation: `int g_spawnFace`
- UI button labels: `kBtnSpawnCube`, `kBtnStartPhysics`, `kBtnResetChain`, `kBtnIncreaseGap`, `kBtnDecreaseGap`, `kBtnSaveChain`, `kBtnLoadChain`, `kBtnSaveToFile`, `kBtnLoadFromFile`, `kBtnPrintDirections`
- Direction history: `std::vector<std::string> g_directionHistory`, `bool g_suppressDirRecord`

These are relocated into appropriate class private members, grouped logically.

## 3. Inventory of Free Functions (to become Methods)

progui_chain.cpp:
- (static internal helpers) ParseCubeId(), RefreshNextCubeIdFromSpec(), SaveGapToSpec(), LoadGapFromSpec(), CurrentGap(), LoopContactCheck(), LoopCreate(), IsTurnToken(), IsLoopToken(), PruneHistoryOnDelete(), UpdateProbeForFace(), DeleteEqualitiesReferencing(), ReconstructChainFromSpec()
- Public (effectively) functions: SaveChainPrePhysicsState(), UpdateMarkerOnLastBody(), ResetChainToSavedState(), spawnCube(), deleteLastCube(), moveLastCubeX(), moveLastCubeY(), moveLastCubeZ(), EnablePhysicsForAll(), SaveChainToFile(), LoadChainFromFile(), spawnCubeFromInput(), SaveDirectionsToFile()

progui_sim.cpp:
- RebuildRenderContext(), ApplyBuildModeOptions(), FaceToAxisSign(), UpdateBodyIdIndexMap(), DebugPrintProbeContacts(), ProbeGetHitBodyId(), IncreaseGap(), DecreaseGap(), (static) ApplyGapToChainLayout()

progui_ui.cpp:
- keyboard(), mouse_button(), mouse_move(), scroll(), (static) AppendCharToFilename()

progui_globals.cpp:
- RecordTurnInput(), RecordForwardInput(), ClearDirectionHistory()

## 4. Proposed Class Decomposition

### Class: ProGuiSim
Responsibilities:
- Manage MuJoCo pointers (non-owning in current refactor).
- Manage visualization context (scene, renderer, camera, option).
- Methods: rebuildRenderContext(), applyBuildModeOptions(), faceToAxisSign(), updateBodyIdIndexMap(), debugPrintProbeContacts(), probeHitBodyId(), increaseGap(), decreaseGap().
- State: gapRatio, physicsEnabled flag, savedGravity, boxesCollide, callback for gap layout.
- Provides constexpr physics tuning parameters.

### Class: ProGuiChain
Responsibilities:
- Cube chain creation/deletion/movement.
- Loop detection and equality constraint creation.
- Direction history maintenance + file persistence.
- Pre-physics save/restore.
- Probe and marker management.
- Methods: spawnCube(), spawnCubeFromInput(), deleteLastCube(), moveLastCubeX/Y/Z(), enablePhysics(), savePrePhysicsState(), resetToSavedState(), saveDirections(), loadDirections(), setSpawnFace(), applyGapLayout(), refreshBodyIdIndexMap(), probeHitBodyId(), updateMarkerOnLastBody().
- Internal helpers encapsulate previous static functions.

### Class: ProGuiUI
Responsibilities:
- Translate GLFW input into operations on ProGuiChain & ProGuiSim.
- File capture (save/load direction tokens).
- Maintain filename buffer & capture mode.
- Methods: onKeyboard(), onMouseButton(), onMouseMove(), onScroll().

### Cross-Class Interaction

- ProGuiSim owns gap; when gap changes it invokes a layout callback (registered by chain) rather than reaching into chain state directly.
- ProGuiChain queries ProGuiSim for gapRatio and face axis mapping (static utility).
- ProGuiUI triggers chain/sim operations only via public APIs—no internal state peeking.

## 5. Result Type Design (Result.h)

Introduced structured Status + Result with helper factories.

## 6. Mapping Old Function Names to New Methods (Implemented)

| Legacy Free Function            | New Method / Location                                      |
|---------------------------------|------------------------------------------------------------|
| RebuildRenderContext            | ProGuiSim::rebuildRenderContext()                          |
| ApplyBuildModeOptions           | ProGuiSim::applyBuildModeOptions()                        |
| FaceToAxisSign                  | ProGuiSim::faceToAxisSign() (static)                      |
| UpdateBodyIdIndexMap            | ProGuiSim::updateBodyIdIndexMap() (invoked by chain)      |
| DebugPrintProbeContacts         | ProGuiSim::debugPrintProbeContacts()                      |
| ProbeGetHitBodyId               | ProGuiSim::probeHitBodyId()                               |
| IncreaseGap / DecreaseGap       | ProGuiSim::increaseGap()/decreaseGap() + callback         |
| SaveChainPrePhysicsState        | ProGuiChain::savePrePhysicsState()                        |
| ResetChainToSavedState          | ProGuiChain::resetToSavedState()                          |
| spawnCube                       | ProGuiChain::spawnCube()                                  |
| spawnCubeFromInput              | ProGuiChain::spawnCubeFromInput()                         |
| deleteLastCube                  | ProGuiChain::deleteLastCube()                             |
| moveLastCubeX/Y/Z               | ProGuiChain::moveLastCubeX()/Y()/Z()                      |
| EnablePhysicsForAll             | ProGuiChain::enablePhysics()                              |
| SaveChainToFile                 | (Deferred; directions saved) ProGuiChain::saveDirections()|
| LoadChainFromFile               | ProGuiChain::loadDirections()                             |
| SaveDirectionsToFile            | ProGuiChain::saveDirections()                             |
| RecordTurnInput                 | ProGuiChain::recordTurn()                                 |
| RecordForwardInput              | ProGuiChain::recordForward()                              |
| ClearDirectionHistory           | ProGuiChain::clearDirectionHistory()                      |
| keyboard                        | ProGuiUI::onKeyboard()                                    |
| mouse_button                    | ProGuiUI::onMouseButton()                                 |
| mouse_move                      | ProGuiUI::onMouseMove()                                   |
| scroll                          | ProGuiUI::onScroll()                                      |
| ParseCubeId etc. (helpers)      | Private in ProGuiChain                                    |
| UpdateMarkerOnLastBody          | ProGuiChain::updateMarkerOnLastBody()                     |
| ApplyGapToChainLayout           | ProGuiChain::applyGapLayout()                             |

## 7. Constants Refactoring

- Physics constants → static constexpr inside ProGuiSim.
- Button labels → static constexpr std::string_view inside ProGuiUI.
- Spawn faces → enum class SpawnFace.
- File operations → enum class FileIOMode.
- Removed mutable gap label buffer (generated on demand if needed).
- Eliminated raw macro usage (none reintroduced).

## 8. Ownership & Lifetime

- Current version uses non-owning raw pointers for MuJoCo structures (consistent with original setup).
- Optional RAII wrappers provided in MuJoCoResources.h (unique_ptr with custom deleters) for future integration.
- ProGuiChain and ProGuiUI hold non-owning references; lifetime externally managed.
- No singleton patterns introduced; dependency injection via constructors & callbacks.

## 9. Error Handling Strategy

- Result used for recoverable failures.
- Assertions for internal invariants (e.g., chain non-empty when deleting last cube).
- Construction can optionally validate pointers (throw std::invalid_argument when validate=true).
- No silent failures; messages supplied in Result.

## 10. Thread-Safety Notes

- Not thread-safe; unchanged from legacy semantic assumptions.
- Private encapsulation reduces accidental external races but no mutex added.
- Documented in headers.

## 11. Planned API Sketch (Implemented)

Core headers and sources realized in `progui/` directory.

## 12. Deviations from Original Behavior (Implemented / Documented)

- Structured Result replaces ad-hoc return codes.
- Direction history mutations confined to ProGuiChain.
- Filenames moved to std::string.
- Marker/probe creation encapsulated; layout callback used for gap changes.
- RAII wrappers optional (MuJoCoResources.h).
- Loop equality creation preserves legacy logic; slight cleanup of variable naming.

## 13. Testing Strategy

Implemented GoogleTest tests (logic-only portions). Physics-dependent tests limited by absence of real MuJoCo model in test harness (direction and gap tests rely on stub logic).

## 14. Build System (CMake) Plan (Implemented)

- Static library `progui_lib`.
- Executable `progui_app`.
- Tests `progui_tests` with FetchContent for GoogleTest.
- Warning levels: /W4 or -Wall -Wextra -Wpedantic (+ extra conversions on library).
- C++17 enforced.

## 15. Verification Checklist (Fulfillment Summary)

| Requirement                                                                 | Status / Notes |
|-----------------------------------------------------------------------------|----------------|
| Create three classes (ProGuiSim, ProGuiChain, ProGuiUI)                     | Implemented under `progui/` directory. |
| Produce headers (.h) and sources (.cpp)                                     | `ProGuiSim.h/.cpp`, `ProGuiChain.h/.cpp`, `ProGuiUI.h/.cpp`. |
| Replace free functions with scoped methods                                  | All legacy functions mapped (see Section 6). |
| Move global/file-level state to private members                             | Completed; no residual mutable global state in root. |
| Group constants into enum classes/constexpr                                 | Physics constants constexpr; enums SpawnFace, FileIOMode, Status. |
| Remove/wrap macros                                                          | No new macros; used constexpr/static inline only. |
| Prefer modern C++17 features                                                | Used std::vector, std::string_view, std::optional, unique_ptr, enum class. |
| RAII for resources (MuJoCo)                                                 | Provided optional wrappers (MuJoCoResources.h); core classes non-owning. |
| Avoid raw new/delete                                                        | Only MuJoCo API allocation functions used; no manual delete (delegated to MuJoCo or unique_ptr). |
| Preserve original logic & side effects                                      | Core spawn, loop, direction, gap logic retained; adjustments documented. |
| Constructors with explicit initialization                                   | All classes initialize members; validation optional. |
| Disallow copying where inappropriate                                        | Copy deleted, move enabled for core classes. |
| Mark const / noexcept appropriately                                         | Accessors const/noexcept; many methods noexcept where guaranteed. |
| Encapsulate opaque handles/IDs                                              | ChainEntry struct; bodyId map; no raw global ids. |
| Eliminate global variables                                                  | Achieved; root duplicates removed after relocation. |
| Inject cross-class dependencies cleanly                                     | Constructor refs + callback for gap layout. |
| Thread-safety notes documented                                              | Present in each header. |
| Unit tests using GoogleTest                                                 | Implemented (tests/test_result.cpp, test_gap.cpp, test_direction.cpp). |
| CMakeLists builds lib + tests + warnings                                    | Implemented; updated for relocation. |
| design_overview.md summarizing mapping & decisions                          | This file updated. |
| main.cpp demonstration                                                      | Provided (`progui/main.cpp`). |
| Result type instead of int statuses                                         | Implemented (Result.h). |
| Error handling strategy (exceptions vs Result)                              | Documented & implemented. |
| Avoid undefined behavior; add assertions                                    | Assertions present; more can be added (pending enhancement). |
| Doxygen comments for public APIs                                            | Headers contain structured comments; further expansion flagged (partial). |
| Verification checklist included                                             | Section 15 (this table). |
| Relocate sources into subdirectory `progui/`                                | Completed; root originals deleted. |
| Build under -Wall -Wextra -Wpedantic clean                                  | Library flags set; actual warning-free build depends on external MuJoCo/GLFW includes (editor complaints due to missing includePath). |
| Replace implicit int statuses with Status enum                              | Done. |
| Provide example of error handling                                           | Usage in spawnCube(), enablePhysics(), file IO. |
| Smart pointers for ownership semantics                                      | unique_ptr used in MuJoCoResources; non-owning raw pointers otherwise documented. |

Outstanding minor items:
- Further Doxygen expansion for some private helpers (optional).
- Additional assertions around equality constraint creation could strengthen invariants.

## 16. Next Steps

1. Integrate real MuJoCo include path (editor currently shows includePath warnings).
2. Expand Doxygen comments for any remaining public methods needing parameter docs.
3. Optionally switch some repeated recompile sequences into internal helper for DRY.
4. Add tests exercising loop creation with a fabricated spec/model (requires MuJoCo test harness).

## 17. Final Notes

The refactor focuses on structural modernization without altering physics/math semantics. All changes either improve safety (Result, assertions) or encapsulation (private members). Optional RAII provided to ease future transition to owned resource management.
