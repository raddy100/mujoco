#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declaration — full definition in chainmaker_ipc.cpp
struct IpcServer;

// ---------------------------------------------------------------------------
// SpawnFace — which face of the head block to extend from
// ---------------------------------------------------------------------------

enum SpawnFace : int {
    FACE_POS_X = 0,  // +X direction
    FACE_NEG_X = 1,  // -X direction
    FACE_POS_Y = 2,  // +Y direction
    FACE_NEG_Y = 3,  // -Y direction
    FACE_POS_Z = 4,  // +Z direction
    FACE_NEG_Z = 5,  // -Z direction
};

constexpr int kNumFaces = 6;

// ---------------------------------------------------------------------------
// IVec3 — integer 3D grid coordinate
// ---------------------------------------------------------------------------

struct IVec3 {
    int x = 0, y = 0, z = 0;

    bool operator==(const IVec3& o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator!=(const IVec3& o) const { return !(*this == o); }

    IVec3 operator+(const IVec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    IVec3 operator-(const IVec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    IVec3 operator-() const { return {-x, -y, -z}; }
};

struct IVec3Hash {
    std::size_t operator()(const IVec3& v) const {
        std::size_t h = std::hash<int>{}(v.x);
        h ^= std::hash<int>{}(v.y) * 2654435761u;
        h ^= std::hash<int>{}(v.z) * 40503u;
        return h;
    }
};

// ---------------------------------------------------------------------------
// Direction helpers
// ---------------------------------------------------------------------------

inline void FaceToAxisSign(SpawnFace face, int& axis, int& sign) {
    axis = static_cast<int>(face) / 2;   // 0,1→0  2,3→1  4,5→2
    sign = (static_cast<int>(face) % 2 == 0) ? +1 : -1;
}

inline SpawnFace AxisSignToFace(int axis, int sign) {
    return static_cast<SpawnFace>(axis * 2 + (sign < 0 ? 1 : 0));
}

inline IVec3 FaceToOffset(SpawnFace face) {
    int axis, sign;
    FaceToAxisSign(face, axis, sign);
    IVec3 off{0, 0, 0};
    (&off.x)[axis] = sign;
    return off;
}

inline SpawnFace OppositeFace(SpawnFace face) {
    return static_cast<SpawnFace>(static_cast<int>(face) ^ 1);
}

inline const char* FaceName(SpawnFace face) {
    static const char* names[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
    int idx = static_cast<int>(face);
    if (idx < 0 || idx >= kNumFaces) return "?";
    return names[idx];
}

// ---------------------------------------------------------------------------
// GridCell — occupancy record for one grid position
// ---------------------------------------------------------------------------

struct GridCell {
    IVec3 pos;

    // Which chain occupies each axis (-1 = unoccupied)
    int chain_on_axis[3] = {-1, -1, -1};  // [X, Y, Z]

    bool is_turn         = false;
    int  turn_entry_axis = -1;
    int  turn_exit_axis  = -1;

    int ChainCount() const {
        int n = 0;
        for (int i = 0; i < 3; i++) {
            if (chain_on_axis[i] >= 0) n++;
        }
        return n;
    }

    bool IsJunction() const { return ChainCount() >= 2; }

    bool IsAxisFree(int axis) const {
        return axis >= 0 && axis < 3 && chain_on_axis[axis] < 0;
    }
};

// ---------------------------------------------------------------------------
// Chain — ordered list of grid positions forming one chain
// ---------------------------------------------------------------------------

struct Chain {
    int         id             = -1;
    std::string name;
    float       color[4]       = {1, 1, 1, 1};
    std::vector<IVec3> blocks;
    SpawnFace   head_direction = FACE_POS_X;
    // Tracks how many blocks each PlaceBlock call added (for atomic undo).
    std::vector<int>   undo_stack;

    IVec3 Head() const { return blocks.back(); }
    int   Size() const { return static_cast<int>(blocks.size()); }
    bool  Empty() const { return blocks.empty(); }
};

// ---------------------------------------------------------------------------
// AppMode
// ---------------------------------------------------------------------------

enum class AppMode {
    BUILD,               // normal block placement
    NEW_CHAIN_PICK,      // waiting for user to click a junction block
    NEW_CHAIN_DIRECTION, // waiting for direction pick (not currently used as separate state)
    SIMULATE,            // physics running (Phase 2)
};

// ---------------------------------------------------------------------------
// SimPreset — simulation speed / accuracy trade-off selector
// ---------------------------------------------------------------------------

enum SimPreset {
    SIM_ACCURATE = 0,  // Newton + box collision: slowest, most accurate
    SIM_BALANCED = 1,  // CG    + box collision: ~5x faster, same geometry
    SIM_FAST     = 2,  // CG    + sphere:        ~11x faster, inscribed sphere
};

constexpr int kNumSimPresets = 3;

// ---------------------------------------------------------------------------
// WeldLevel — straight-run weld policy
// ---------------------------------------------------------------------------
// Weld constraints between consecutive non-turn blocks eliminate the ball-
// joint DOF between them, treating straight runs as rigid rods.  This
// dramatically reduces DOF (and solve time) for long straight chains while
// remaining physically justified when string tension is high.
//
// Rules:
//   - A weld is added between blocks[i] and blocks[i+1] only when NEITHER
//     block is a turn block.
//   - Turn blocks and their immediate neighbors are never welded.
//   - WELD_PARTIAL reserved for future implementation.
enum WeldLevel {
    WELD_NONE = 0,  // No straight-run welds (current default) — maximum DOF
    WELD_FULL = 2,  // Weld every eligible consecutive pair — minimum DOF
};
constexpr int kNumWeldLevels = 2;  // NONE and FULL only

// ---------------------------------------------------------------------------
// ChainWorld — central data model
// ---------------------------------------------------------------------------

struct ChainWorld {
    std::unordered_map<IVec3, GridCell, IVec3Hash> grid;
    std::vector<Chain> chains;
    int active_chain_id = -1;

    double bead_size  = 0.05;   // cube edge length in metres
    double gap_ratio  = 0.05;   // gap as fraction of bead_size
    SimPreset sim_preset  = SIM_ACCURATE;  // saved/loaded with file
    WeldLevel weld_level  = WELD_NONE;     // saved/loaded with file
    bool      root_fixed  = true;          // chain-0 root body: fixed=no free joint (default), free=full 6-DOF

    double CellStride() const { return bead_size * (1.0 + gap_ratio); }
    double Gap()        const { return bead_size * gap_ratio; }
    double HalfSize()   const { return bead_size / 2.0; }

    void GridToWorld(const IVec3& g, double out[3]) const {
        double s = CellStride();
        out[0] = g.x * s;
        out[1] = g.y * s;
        out[2] = g.z * s + bead_size;  // lifted off floor by one bead_size
    }

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

// ---------------------------------------------------------------------------
// AppState — single global application state
// ---------------------------------------------------------------------------

struct AppState {
    ChainWorld world;
    AppMode    mode = AppMode::BUILD;

    // New-chain workflow
    IVec3 junction_pick;
    bool  junction_picked = false;

    // Hover tracking for NEW_CHAIN_PICK mode
    IVec3 hovered_block;
    bool  block_hovered = false;

    // MuJoCo visualization (Build Stage — model is NULL)
    mjvCamera  cam;
    mjvOption  opt;
    mjvScene   scn;
    mjrContext con;

    // Minimal model used solely to initialize built-in geometry VBOs in the
    // render context during build stage (mjr_makeContext(nullptr,...) skips
    // makeBuiltin(), so no box/sphere/arrow VBOs would exist).
    mjModel* build_model = nullptr;

    // MuJoCo physics (Simulation Stage, Phase 2)
    mjModel* sim_model = nullptr;
    mjData*  sim_data  = nullptr;
    mjSpec*  sim_spec  = nullptr;

    // UI
    mjUI      ui;
    mjuiState uistate;
    int       ui_width = 220;

    // Mouse
    bool   mouse_left   = false;
    bool   mouse_middle = false;
    bool   mouse_right  = false;
    double mouse_lastx  = 0;
    double mouse_lasty  = 0;

    // Profiler overlay toggle
    bool show_profiler = false;

    // Video recording (F9 key or IPC start_recording/stop_recording)
    bool            is_recording    = false;
    char            record_path[512] = "";
    FILE*           record_pipe     = nullptr;
    int             record_width    = 0;
    int             record_height   = 0;
    unsigned char*  record_pixels   = nullptr;

    // Status / overlay text
    char status_text[512] = "";

    // Request UI rebuild on next frame (e.g. after Sim Speed button cycles preset)
    bool needs_ui_rebuild = false;

    // File I/O capture
    char io_filename[256] = "";
    int  io_mode          = 0;   // 0=none, 1=save, 2=load

    // GLFW window pointer (set in main)
    GLFWwindow* window = nullptr;

    // IPC test server (only active when launched with --test flag)
    bool        test_mode = false;
    IpcServer*  ipc       = nullptr;  // opaque; defined in chainmaker_ipc.cpp
};

// ---------------------------------------------------------------------------
// Physics / rendering constants
// ---------------------------------------------------------------------------

constexpr double kJointDamping       = 0.15;
constexpr double kSolref[2]          = {0.005, 2.0};   // near-rigid, overdamped (no bounce)
constexpr double kSolimp[5]          = {0.95, 0.999, 0.001, 0.5, 2.0};  // near-impenetrable wall
constexpr double kGeomMargin         = 0.0;
constexpr double kDefaultBeadSize    = 0.05;   // 5 cm
constexpr double kDefaultGapRatio    = 0.05;
constexpr double kJunctionSolref[2]  = {0.005, 1.0};
constexpr int    kMinCollisionDist   = 3;
constexpr int    kMaxSceneGeoms      = 20000;

// ---------------------------------------------------------------------------
// Chain operation result codes
// ---------------------------------------------------------------------------

enum class PlaceResult {
    SUCCESS,
    NO_ACTIVE_CHAIN,
    TARGET_OCCUPIED_INCOMPATIBLE,
    WOULD_REVERSE,
    HEAD_WOULD_BECOME_INVALID_TURN,
};

// ---------------------------------------------------------------------------
// Chain operation declarations (implemented in chainmaker.cpp)
// ---------------------------------------------------------------------------

void CreateFirstChain(ChainWorld& world);
PlaceResult PlaceBlock(ChainWorld& world);
void DeleteLastBlock(ChainWorld& world);
void SetDirection(ChainWorld& world, SpawnFace new_face);
void SwitchChain(ChainWorld& world, int index);
void AdjustGap(ChainWorld& world, double delta);
bool StartNewChainFromBlock(AppState& app, const IVec3& junction_pos);
bool RayPickBlock(const AppState& app, double mouse_x, double mouse_y,
                  int fb_width, int fb_height, IVec3& out);
