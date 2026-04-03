#include "chainmaker_render.h"

#include <cmath>
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// BuildArrowRotation
// Rotates the default +Z-pointing arrow to point along the given face.
// Column-major? No — MuJoCo geom mat is row-major (row = local axis).
// mat[0..2] = local X in world,  mat[3..5] = local Y,  mat[6..8] = local Z.
// We want local Z (the arrow shaft direction) to align with the face offset.
// ---------------------------------------------------------------------------

void BuildArrowRotation(SpawnFace face, mjtNum mat[9]) {
    int axis, sign;
    FaceToAxisSign(face, axis, sign);

    // Desired shaft direction (local Z)
    double shaft[3] = {0, 0, 0};
    shaft[axis] = sign;

    // Choose a stable "up" that is not parallel to shaft
    double up_candidates[3][3] = {{0,0,1},{1,0,0},{0,1,0}};
    double* up = up_candidates[0];
    // If shaft ≈ ±Z, use Y as up
    if (std::abs(shaft[2]) > 0.9) up = up_candidates[1];

    // right = up × shaft (local X)
    double rx = up[1]*shaft[2] - up[2]*shaft[1];
    double ry = up[2]*shaft[0] - up[0]*shaft[2];
    double rz = up[0]*shaft[1] - up[1]*shaft[0];
    double rlen = std::sqrt(rx*rx + ry*ry + rz*rz);
    if (rlen > 1e-12) { rx/=rlen; ry/=rlen; rz/=rlen; }

    // true_up = shaft × right (local Y)
    double ux = shaft[1]*rz - shaft[2]*ry;
    double uy = shaft[2]*rx - shaft[0]*rz;
    double uz = shaft[0]*ry - shaft[1]*rx;

    // Row-major: row0=local X, row1=local Y, row2=local Z
    mat[0] = rx;  mat[1] = ry;  mat[2] = rz;
    mat[3] = ux;  mat[4] = uy;  mat[5] = uz;
    mat[6] = shaft[0]; mat[7] = shaft[1]; mat[8] = shaft[2];
}

// ---------------------------------------------------------------------------
// ComputeBlockColor
// ---------------------------------------------------------------------------

void ComputeBlockColor(const ChainWorld& world, const GridCell& cell, float rgba[4]) {
    int   count = 0;
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

// ---------------------------------------------------------------------------
// AddBuildLights — set up scene lights for the NULL-model build stage.
// mjv_makeLights() requires a valid mjModel, so we add lights manually.
// ---------------------------------------------------------------------------

static void AddBuildLights(AppState& app) {
    mjvScene& scn = app.scn;
    scn.nlight = 0;

    const mjvGLCamera& cam = scn.camera[0];

    // 1. Headlight — follows the camera
    {
        mjvLight& L = scn.lights[scn.nlight++];
        memset(&L, 0, sizeof(L));
        L.id         = -1;
        L.headlight  = 1;
        L.castshadow = 0;
        L.texid      = -1;
        L.type       = mjLIGHT_DIRECTIONAL;
        L.range      = 100.0f;
        L.pos[0] = cam.pos[0];  L.pos[1] = cam.pos[1];  L.pos[2] = cam.pos[2];
        L.dir[0] = cam.forward[0]; L.dir[1] = cam.forward[1]; L.dir[2] = cam.forward[2];
        L.ambient[0]  = L.ambient[1]  = L.ambient[2]  = 0.1f;
        L.diffuse[0]  = L.diffuse[1]  = L.diffuse[2]  = 0.4f;
        L.specular[0] = L.specular[1] = L.specular[2] = 0.3f;
    }

    // 2. Overhead fill light (fixed, pointing downward)
    if (scn.nlight < mjMAXLIGHT) {
        mjvLight& L = scn.lights[scn.nlight++];
        memset(&L, 0, sizeof(L));
        L.id         = 0;
        L.headlight  = 0;
        L.castshadow = 0;
        L.texid      = -1;
        L.type       = mjLIGHT_DIRECTIONAL;
        L.range      = 100.0f;
        L.pos[0] = 0.0f;  L.pos[1] = 0.0f;  L.pos[2] = 5.0f;
        L.dir[0] = 0.0f;  L.dir[1] = 0.0f;  L.dir[2] = -1.0f;
        L.ambient[0]  = L.ambient[1]  = L.ambient[2]  = 0.05f;
        L.diffuse[0]  = L.diffuse[1]  = L.diffuse[2]  = 0.5f;
        L.specular[0] = L.specular[1] = L.specular[2] = 0.2f;
    }
}

// ---------------------------------------------------------------------------
// AddFloorGeom
// ---------------------------------------------------------------------------

void AddFloorGeom(mjvScene& scn, const ChainWorld& /*world*/) {
    if (scn.ngeom >= scn.maxgeom) return;
    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjtNum size[3] = {10.0, 10.0, 0.02};
    mjtNum pos[3]  = {0.0, 0.0, 0.0};
    float  rgba[4] = {0.4f, 0.4f, 0.45f, 1.0f};
    mjv_initGeom(g, mjGEOM_PLANE, size, pos, NULL, rgba);
    scn.ngeom++;
}

// ---------------------------------------------------------------------------
// AddBlockGeom
// ---------------------------------------------------------------------------

void AddBlockGeom(mjvScene& scn, const ChainWorld& world,
                  const IVec3& pos, const GridCell& cell) {
    if (scn.ngeom >= scn.maxgeom) return;

    mjvGeom* g = &scn.geoms[scn.ngeom];
    double   half = world.HalfSize();
    mjtNum   size[3] = {half, half, half};
    mjtNum   wpos[3];
    world.GridToWorld(pos, wpos);

    float rgba[4];
    ComputeBlockColor(world, cell, rgba);

    // Slightly brighten turn blocks for visual distinction
    if (cell.is_turn) {
        rgba[0] = std::min(1.0f, rgba[0] * 1.3f);
        rgba[1] = std::min(1.0f, rgba[1] * 1.3f);
        rgba[2] = std::min(1.0f, rgba[2] * 1.3f);
    }

    mjv_initGeom(g, mjGEOM_BOX, size, wpos, NULL, rgba);
    scn.ngeom++;
}

// ---------------------------------------------------------------------------
// AddGhostGeom
// ---------------------------------------------------------------------------

void AddGhostGeom(mjvScene& scn, const ChainWorld& world, const Chain& chain) {
    if (scn.ngeom >= scn.maxgeom) return;

    IVec3 offset    = FaceToOffset(chain.head_direction);
    IVec3 ghost_pos = chain.Head() + offset;

    // Skip through already-occupied cells so the ghost shows the actual
    // placement position (handles double/triple junction scenarios).
    // RULE: turn blocks are completely off-limits — stop immediately and
    //       show no ghost if a turn block is in the path.
    int max_skip = 20;
    while (world.grid.count(ghost_pos) > 0 && max_skip-- > 0) {
        const GridCell& cell = world.grid.at(ghost_pos);
        if (cell.is_turn) return;   // silent: no ghost beyond a turn block
        ghost_pos = ghost_pos + offset;
    }
    // If we exhausted the skip limit and still hit occupied cells, no ghost
    if (world.grid.count(ghost_pos) > 0) return;
    mjtNum wpos[3];
    world.GridToWorld(ghost_pos, wpos);

    double half = world.HalfSize();
    mjtNum size[3] = {half, half, half};
    float  rgba[4] = {chain.color[0], chain.color[1], chain.color[2], 0.3f};

    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjv_initGeom(g, mjGEOM_BOX, size, wpos, NULL, rgba);  // use solid box with low alpha
    scn.ngeom++;
}

// ---------------------------------------------------------------------------
// AddHeadMarker
// ---------------------------------------------------------------------------

void AddHeadMarker(mjvScene& scn, const ChainWorld& world, const IVec3& head) {
    if (scn.ngeom >= scn.maxgeom) return;

    mjtNum wpos[3];
    world.GridToWorld(head, wpos);

    double sphere_r = world.HalfSize() * 0.35;
    mjtNum size[3]  = {sphere_r, sphere_r, sphere_r};
    float  rgba[4]  = {1.0f, 1.0f, 0.0f, 0.9f};  // yellow

    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjv_initGeom(g, mjGEOM_SPHERE, size, wpos, NULL, rgba);
    scn.ngeom++;
}

// ---------------------------------------------------------------------------
// AddDirectionArrow
// ---------------------------------------------------------------------------

void AddDirectionArrow(mjvScene& scn, const ChainWorld& world,
                       const IVec3& head, SpawnFace face) {
    if (scn.ngeom >= scn.maxgeom) return;

    mjtNum wpos[3];
    world.GridToWorld(head, wpos);

    // Shift arrow start to the face of the block
    int axis, sign;
    FaceToAxisSign(face, axis, sign);
    wpos[axis] += sign * world.HalfSize();

    double shaft = world.CellStride() * 0.6;
    double radius = world.HalfSize() * 0.1;
    mjtNum size[3]  = {shaft, radius, 0};
    float  rgba[4]  = {1.0f, 1.0f, 1.0f, 0.8f};

    mjtNum mat[9];
    BuildArrowRotation(face, mat);

    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjv_initGeom(g, mjGEOM_ARROW, size, wpos, mat, rgba);
    scn.ngeom++;
}

// ---------------------------------------------------------------------------
// AddJunctionHighlight
// ---------------------------------------------------------------------------

void AddJunctionHighlight(mjvScene& scn, const ChainWorld& world,
                          const IVec3& pos) {
    if (scn.ngeom >= scn.maxgeom) return;

    mjtNum wpos[3];
    world.GridToWorld(pos, wpos);

    double half = world.HalfSize() * 1.05;  // slightly larger than the block
    mjtNum size[3] = {half, half, half};
    float  rgba[4] = {1.0f, 1.0f, 0.0f, 0.4f};  // yellow highlight

    mjvGeom* g = &scn.geoms[scn.ngeom];
    mjv_initGeom(g, mjGEOM_BOX, size, wpos, NULL, rgba);
    scn.ngeom++;
}

// ---------------------------------------------------------------------------
// PopulateBuildScene
// ---------------------------------------------------------------------------

void PopulateBuildScene(AppState& app) {
    mjvScene& scn = app.scn;
    scn.ngeom = 0;

    // 0. Lights
    AddBuildLights(app);

    // 1. Floor
    AddFloorGeom(scn, app.world);

    // 2. All occupied grid cells
    for (const auto& [pos, cell] : app.world.grid) {
        AddBlockGeom(scn, app.world, pos, cell);
    }

    // 3. Ghost preview (build mode only)
    if (app.mode == AppMode::BUILD) {
        const Chain* active = app.world.ActiveChain();
        if (active && !active->Empty()) {
            AddGhostGeom(scn, app.world, *active);
        }
    }

    // 4. Head marker
    if (app.mode == AppMode::BUILD || app.mode == AppMode::NEW_CHAIN_PICK) {
        const Chain* active = app.world.ActiveChain();
        if (active && !active->Empty()) {
            AddHeadMarker(scn, app.world, active->Head());
        }
    }

    // 5. Direction arrow (build mode only)
    if (app.mode == AppMode::BUILD) {
        const Chain* active = app.world.ActiveChain();
        if (active && !active->Empty()) {
            AddDirectionArrow(scn, app.world, active->Head(), active->head_direction);
        }
    }

    // 6. Junction highlight (new-chain-pick mode)
    if (app.mode == AppMode::NEW_CHAIN_PICK) {
        if (app.block_hovered && !app.junction_picked) {
            AddJunctionHighlight(scn, app.world, app.hovered_block);
        }
        if (app.junction_picked) {
            AddJunctionHighlight(scn, app.world, app.junction_pick);
        }
    }
}

// ---------------------------------------------------------------------------
// RenderOverlays
// ---------------------------------------------------------------------------

void RenderOverlays(AppState& app, mjrRect viewport) {
    const Chain* active = app.world.ActiveChain();

    // Bottom-left: chain status
    if (active) {
        std::snprintf(app.status_text, sizeof(app.status_text),
            "[%s] Head:(%d,%d,%d) Dir:%s Blocks:%d | Total grid:%zu | Chains:%zu\n"
            "SPACE/C=place DEL=delete N=new-chain 1-9=switch +/-=gap Z/X=up/down P=simulate",
            active->name.c_str(),
            active->Head().x, active->Head().y, active->Head().z,
            FaceName(active->head_direction),
            active->Size(),
            app.world.grid.size(),
            app.world.chains.size());
    } else {
        std::snprintf(app.status_text, sizeof(app.status_text), "(no active chain)");
    }
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                app.status_text, NULL, &app.con);

    // Top-left: mode indicator and file I/O prompt
    if (app.mode == AppMode::NEW_CHAIN_PICK) {
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                    "NEW CHAIN MODE: Click a block to start a new chain (ESC to cancel)",
                    NULL, &app.con);
    } else if (app.io_mode != 0) {
        char io_text[320];
        const char* mode_str = (app.io_mode == 1) ? "Save" : "Load";
        std::snprintf(io_text, sizeof(io_text),
            "%s filename: %s_\n(Enter=confirm  ESC=cancel)", mode_str, app.io_filename);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                    io_text, NULL, &app.con);
    }
}
