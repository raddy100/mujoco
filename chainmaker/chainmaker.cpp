#include "chainmaker.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

// ---------------------------------------------------------------------------
// CreateFirstChain
// ---------------------------------------------------------------------------

void CreateFirstChain(ChainWorld& world) {
    world.grid.clear();
    world.chains.clear();
    world.active_chain_id = -1;

    Chain chain;
    chain.id   = 0;
    chain.name = "Chain A";
    std::memcpy(chain.color, ChainWorld::kPalette[0], sizeof(float) * 4);
    chain.head_direction = FACE_POS_X;

    IVec3 origin{0, 0, 0};
    chain.blocks.push_back(origin);

    GridCell cell;
    cell.pos = origin;
    // The first block's axis comes from the initial head_direction
    int axis, sign;
    FaceToAxisSign(chain.head_direction, axis, sign);
    cell.chain_on_axis[axis] = chain.id;

    world.grid[origin] = cell;
    world.chains.push_back(chain);
    world.active_chain_id = 0;
}

// ---------------------------------------------------------------------------
// PlaceBlock
// ---------------------------------------------------------------------------

PlaceResult PlaceBlock(ChainWorld& world) {
    Chain* chain = world.ActiveChain();
    if (!chain || chain->Empty()) return PlaceResult::NO_ACTIVE_CHAIN;

    IVec3 head = chain->Head();
    SpawnFace dir = chain->head_direction;
    IVec3 fwd = FaceToOffset(dir);

    // Guard against 180° reversal
    if (chain->Size() >= 2) {
        IVec3 prev     = chain->blocks[chain->Size() - 2];
        IVec3 back_dir = prev - head;
        if (fwd == back_dir) return PlaceResult::WOULD_REVERSE;
    }

    int target_axis, target_sign;
    FaceToAxisSign(dir, target_axis, target_sign);

    // Guard: head can only become a turn if it's not already a junction
    bool head_turns = false;
    if (chain->Size() >= 2) {
        IVec3 prev      = chain->blocks[chain->Size() - 2];
        IVec3 entry_vec = head - prev;
        head_turns = (entry_vec != fwd);
        if (head_turns) {
            GridCell& head_cell = world.grid[head];
            if (head_cell.IsJunction())
                return PlaceResult::HEAD_WOULD_BECOME_INVALID_TURN;
        }
    }

    // Walk forward collecting ALL consecutive occupied cells as junctions.
    // The first free cell is where the new bead lands.
    static constexpr int kMaxJunctionSkip = 20;
    std::vector<IVec3> junctions;
    IVec3 cur = head + fwd;

    while (world.grid.count(cur) > 0 && (int)junctions.size() < kMaxJunctionSkip) {
        GridCell& cell = world.grid[cur];
        if (cell.is_turn || !cell.IsAxisFree(target_axis))
            return PlaceResult::TARGET_OCCUPIED_INCOMPATIBLE;
        junctions.push_back(cur);
        cur = cur + fwd;
    }

    // If we still hit an occupied cell after the skip limit, bail out
    if (world.grid.count(cur) > 0)
        return PlaceResult::TARGET_OCCUPIED_INCOMPATIBLE;

    IVec3 new_bead = cur;

    // --- Commit all changes (no rollback needed: all checks passed above) ---

    // Mark turn at head
    if (head_turns) {
        GridCell& head_cell = world.grid[head];
        IVec3 prev      = chain->blocks[chain->Size() - 2];
        IVec3 entry_vec = head - prev;
        head_cell.is_turn = true;
        for (int ax = 0; ax < 3; ax++) {
            if ((&entry_vec.x)[ax] != 0) head_cell.turn_entry_axis = ax;
            if ((&fwd.x)[ax]       != 0) head_cell.turn_exit_axis  = ax;
        }
    }

    // Claim all junction cells' axes and record them in the chain path
    for (const IVec3& j : junctions) {
        world.grid[j].chain_on_axis[target_axis] = chain->id;
        chain->blocks.push_back(j);
    }

    // Place the new bead
    GridCell cell;
    cell.pos = new_bead;
    cell.chain_on_axis[target_axis] = chain->id;
    world.grid[new_bead] = cell;
    chain->blocks.push_back(new_bead);

    // Record how many blocks this PlaceBlock call added for atomic undo
    chain->undo_stack.push_back(static_cast<int>(junctions.size()) + 1);

    return PlaceResult::SUCCESS;
}

// ---------------------------------------------------------------------------
// DeleteLastBlock
// ---------------------------------------------------------------------------

void DeleteLastBlock(ChainWorld& world) {
    Chain* chain = world.ActiveChain();
    if (!chain || chain->Size() <= 1) return;  // keep at least 1 block

    // Determine how many blocks to remove this press (1 normally, N for multi-junction)
    int count = 1;
    if (!chain->undo_stack.empty()) {
        count = chain->undo_stack.back();
        chain->undo_stack.pop_back();
        // Clamp to how many blocks are actually removable
        count = std::min(count, chain->Size() - 1);
    }

    for (int k = 0; k < count; ++k) {
        if (chain->Size() <= 1) break;

        IVec3 last = chain->blocks.back();
        IVec3 prev = chain->blocks[chain->Size() - 2];
        chain->blocks.pop_back();

        // Infer the axis the deleted block occupied from the travel direction
        IVec3 delta = last - prev;
        int axis_to_clear = (delta.y != 0) ? 1 : (delta.z != 0) ? 2 : 0;

        auto it = world.grid.find(last);
        if (it != world.grid.end()) {
            GridCell& cell = it->second;
            if (cell.chain_on_axis[axis_to_clear] == chain->id)
                cell.chain_on_axis[axis_to_clear] = -1;

            if (cell.ChainCount() == 0) {
                world.grid.erase(it);
            } else {
                // Other chain(s) still occupy this cell; clear turn data
                cell.is_turn         = false;
                cell.turn_entry_axis = -1;
                cell.turn_exit_axis  = -1;
            }
        }
    }

    // Un-turn the new head (cleared once, after all pops)
    if (!chain->Empty()) {
        IVec3 new_head = chain->Head();
        auto hit = world.grid.find(new_head);
        if (hit != world.grid.end()) {
            hit->second.is_turn         = false;
            hit->second.turn_entry_axis = -1;
            hit->second.turn_exit_axis  = -1;
        }
    }
}

// ---------------------------------------------------------------------------
// SetDirection
// ---------------------------------------------------------------------------

void SetDirection(ChainWorld& world, SpawnFace new_face) {
    Chain* chain = world.ActiveChain();
    if (!chain || chain->Empty()) return;

    // Prevent 180° reversal
    if (chain->Size() >= 2) {
        IVec3 head      = chain->Head();
        IVec3 prev      = chain->blocks[chain->Size() - 2];
        IVec3 entry_dir = head - prev;
        IVec3 reverse   = -entry_dir;
        if (FaceToOffset(new_face) == reverse) return;  // silently reject
    }

    chain->head_direction = new_face;
}

// ---------------------------------------------------------------------------
// SwitchChain
// ---------------------------------------------------------------------------

void SwitchChain(ChainWorld& world, int index) {
    if (index >= 0 && index < (int)world.chains.size())
        world.active_chain_id = index;
}

// ---------------------------------------------------------------------------
// AdjustGap
// ---------------------------------------------------------------------------

void AdjustGap(ChainWorld& world, double delta) {
    world.gap_ratio = std::max(0.0, std::min(0.5, world.gap_ratio + delta));
}

// ---------------------------------------------------------------------------
// StartNewChainFromBlock
// ---------------------------------------------------------------------------

bool StartNewChainFromBlock(AppState& app, const IVec3& junction_pos) {
    auto it = app.world.grid.find(junction_pos);
    if (it == app.world.grid.end()) return false;

    GridCell& cell = it->second;
    if (cell.is_turn) return false;  // can't branch from a turn block

    // Find a free axis
    int free_axis = -1;
    for (int ax = 0; ax < 3; ax++) {
        if (cell.IsAxisFree(ax)) { free_axis = ax; break; }
    }
    if (free_axis < 0) return false;  // all axes occupied

    int new_id = (int)app.world.chains.size();

    Chain chain;
    chain.id = new_id;

    // Auto-generate name
    chain.name  = "Chain ";
    chain.name += (char)('A' + (new_id % 26));
    if (new_id >= 26) chain.name += std::to_string(new_id / 26);

    // Assign palette color
    int ci = new_id % ChainWorld::kPaletteSize;
    std::memcpy(chain.color, ChainWorld::kPalette[ci], sizeof(float) * 4);

    chain.head_direction = AxisSignToFace(free_axis, +1);

    // The junction block is the first block of this chain
    chain.blocks.push_back(junction_pos);
    cell.chain_on_axis[free_axis] = new_id;

    app.world.chains.push_back(chain);
    app.world.active_chain_id = new_id;
    app.mode = AppMode::BUILD;

    return true;
}

// ---------------------------------------------------------------------------
// RayPickBlock — ray-AABB intersection against all grid cells
// ---------------------------------------------------------------------------

// Slab ray-AABB test; returns true and fills t if hit
static bool RayAABBIntersect(const double ro[3], const double rd[3],
                              double minx, double miny, double minz,
                              double maxx, double maxy, double maxz,
                              double& t) {
    double tmin = 0.0, tmax = 1e30;

    double bounds_min[3] = {minx, miny, minz};
    double bounds_max[3] = {maxx, maxy, maxz};

    for (int i = 0; i < 3; i++) {
        if (std::abs(rd[i]) < 1e-12) {
            if (ro[i] < bounds_min[i] || ro[i] > bounds_max[i]) return false;
        } else {
            double inv = 1.0 / rd[i];
            double t0  = (bounds_min[i] - ro[i]) * inv;
            double t1  = (bounds_max[i] - ro[i]) * inv;
            if (t0 > t1) std::swap(t0, t1);
            tmin = std::max(tmin, t0);
            tmax = std::min(tmax, t1);
            if (tmin > tmax) return false;
        }
    }
    t = tmin;
    return true;
}

bool RayPickBlock(const AppState& app, double mouse_x, double mouse_y,
                  int fb_width, int fb_height, IVec3& out) {
    if (fb_width <= 0 || fb_height <= 0) return false;

    // Reconstruct camera ray from mjvCamera (free camera mode)
    const mjvCamera& cam = app.cam;

    // Camera spherical → cartesian
    double az  = cam.azimuth  * mjPI / 180.0;
    double el  = cam.elevation * mjPI / 180.0;
    double dist = cam.distance;

    // Camera position
    double cx = cam.lookat[0] - dist * std::cos(el) * std::sin(az);
    double cy = cam.lookat[1] - dist * std::cos(el) * std::cos(az);
    double cz = cam.lookat[2] + dist * std::sin(el);

    // Camera axes
    // Forward = lookat - eye (normalized)
    double fwd[3] = {
        cam.lookat[0] - cx,
        cam.lookat[1] - cy,
        cam.lookat[2] - cz
    };
    double fwd_len = std::sqrt(fwd[0]*fwd[0] + fwd[1]*fwd[1] + fwd[2]*fwd[2]);
    if (fwd_len < 1e-12) return false;
    for (int i = 0; i < 3; i++) fwd[i] /= fwd_len;

    // Up vector = global Z (MuJoCo convention)
    double up[3] = {0, 0, 1};

    // Right = fwd × up (normalized)
    double right[3] = {
        fwd[1]*up[2] - fwd[2]*up[1],
        fwd[2]*up[0] - fwd[0]*up[2],
        fwd[0]*up[1] - fwd[1]*up[0]
    };
    double right_len = std::sqrt(right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
    if (right_len < 1e-12) return false;
    for (int i = 0; i < 3; i++) right[i] /= right_len;

    // Recompute true up = right × fwd
    double true_up[3] = {
        right[1]*fwd[2] - right[2]*fwd[1],
        right[2]*fwd[0] - right[0]*fwd[2],
        right[0]*fwd[1] - right[1]*fwd[0]
    };

    // NDC coordinates from pixel
    double ndx = (2.0 * mouse_x / fb_width)  - 1.0;
    double ndy = 1.0 - (2.0 * mouse_y / fb_height);  // flip Y

    // Field of view (MuJoCo default ~45 deg)
    double fov_y  = 45.0 * mjPI / 180.0;
    double aspect = (double)fb_width / fb_height;
    double tan_fov = std::tan(fov_y * 0.5);

    // Ray direction in world space
    double rd[3] = {
        fwd[0] + ndx * aspect * tan_fov * right[0] + ndy * tan_fov * true_up[0],
        fwd[1] + ndx * aspect * tan_fov * right[1] + ndy * tan_fov * true_up[1],
        fwd[2] + ndx * aspect * tan_fov * right[2] + ndy * tan_fov * true_up[2]
    };
    double rd_len = std::sqrt(rd[0]*rd[0] + rd[1]*rd[1] + rd[2]*rd[2]);
    if (rd_len < 1e-12) return false;
    for (int i = 0; i < 3; i++) rd[i] /= rd_len;

    double ro[3] = {cx, cy, cz};

    double closest_t = 1e30;
    bool   found     = false;

    for (const auto& [pos, cell] : app.world.grid) {
        double wpos[3];
        app.world.GridToWorld(pos, wpos);
        double half = app.world.HalfSize();

        double t = 0;
        if (RayAABBIntersect(ro, rd,
                             wpos[0]-half, wpos[1]-half, wpos[2]-half,
                             wpos[0]+half, wpos[1]+half, wpos[2]+half, t)) {
            if (t > 0.0 && t < closest_t) {
                closest_t = t;
                out = pos;
                found = true;
            }
        }
    }
    return found;
}
