#include "chainmaker_compile.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

// Global contact filter data
std::vector<std::pair<int,int>> g_body_chain_info;

// ---------------------------------------------------------------------------
// ComputeConeLimit — adapted from progui_chain.cpp
// ---------------------------------------------------------------------------

double ComputeConeLimit(double half_size, double gap) {
    const double a     = half_size;
    const double c     = 1.0 + (a > 0.0 ? (gap / a) : 0.0);
    const double root2 = std::sqrt(2.0);

    double phi = 0.0;
    if (c >= root2) {
        phi = mjPI / 2.0;
    } else {
        double s = c / root2;
        s   = std::max(0.0, std::min(1.0, s));
        phi = std::asin(s) - (mjPI / 4.0);
    }

    const double th_min = mjPI * 5.0  / 180.0;
    const double th_max = mjPI * 80.0 / 180.0;
    phi = std::max(th_min, std::min(th_max, phi));

    return phi * 180.0 / mjPI;  // degrees
}

// ---------------------------------------------------------------------------
// ChainmakerContactFilter
// ---------------------------------------------------------------------------

int ChainmakerContactFilter(const mjModel* m, mjData* /*d*/,
                             int geom1, int geom2) {
    int body1 = m->geom_bodyid[geom1];
    int body2 = m->geom_bodyid[geom2];

    // World body (floor) always collides
    if (body1 == 0 || body2 == 0) return 0;

    if (body1 >= (int)g_body_chain_info.size() ||
        body2 >= (int)g_body_chain_info.size()) return 0;

    auto [chain1, pos1] = g_body_chain_info[body1];
    auto [chain2, pos2] = g_body_chain_info[body2];

    // Different chains always collide
    if (chain1 != chain2 || chain1 < 0 || chain2 < 0) return 0;

    // Same chain: filter if neighbours (distance < kMinCollisionDist)
    int distance = std::abs(pos1 - pos2);
    return (distance < kMinCollisionDist) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// CompileWorld
// ---------------------------------------------------------------------------

CompileResult CompileWorld(const ChainWorld& world) {
    CompileResult res;

    if (world.chains.empty()) {
        res.error = "No chains to compile";
        return res;
    }

    // --- Create spec ---
    mjSpec* spec = mj_makeSpec();
    if (!spec) { res.error = "mj_makeSpec() failed"; return res; }
    res.spec = spec;

    // Configure solver options for performance
    // CG solver: 5-10x faster than Newton for large contact systems.
    // Newton builds and factors full KKT matrices per iteration O(n^2+m^2);
    // CG iterates via sparse matrix-vector products — much cheaper when
    // ncon is large (40+ floor contacts with 40 blocks).
    spec->option.jacobian   = mjJAC_SPARSE;
    spec->option.solver     = mjSOL_CG;           // Newton→CG: ~5x per-step speedup
    spec->option.integrator = mjINT_IMPLICITFAST;
    spec->option.timestep   = 0.005;              // 0.002→0.005: stable with IMPLICITFAST
    spec->option.iterations = 15;                 // 20→15: CG converges faster than Newton
    spec->option.tolerance  = 1e-6;

    // --- World body ---
    mjsBody* world_body = mjs_findBody(spec, "world");
    if (!world_body) { res.error = "world body not found"; return res; }

    // --- Floor ---
    {
        // Texture
        mjsTexture* tex = mjs_addTexture(spec);
        mjs_setName(tex->element, "floor_tex");
        tex->type    = mjTEXTURE_2D;
        tex->builtin = mjBUILTIN_CHECKER;
        tex->mark    = mjMARK_EDGE;
        tex->rgb1[0] = 0.55f; tex->rgb1[1] = 0.55f; tex->rgb1[2] = 0.55f;
        tex->rgb2[0] = 0.06f; tex->rgb2[1] = 0.10f; tex->rgb2[2] = 0.35f;
        tex->width   = 512;   tex->height  = 512;    tex->nchannel = 3;

        // Material
        mjsMaterial* mat = mjs_addMaterial(spec, nullptr);
        mjs_setName(mat->element, "floor_mat");
        mat->texrepeat[0] = 12.0f;
        mat->texrepeat[1] = 12.0f;
        mjs_setInStringVec(mat->textures, mjTEXROLE_RGB, "floor_tex");

        // Geom
        mjsGeom* floor = mjs_addGeom(world_body, nullptr);
        floor->type     = mjGEOM_PLANE;
        floor->size[0]  = 25.0; floor->size[1] = 25.0; floor->size[2] = 0.1;
        mjs_setString(floor->material, "floor_mat");
        floor->rgba[0] = 0.5f; floor->rgba[1] = 0.5f;
        floor->rgba[2] = 0.5f; floor->rgba[3] = 1.0f;
        floor->contype     = 1;
        floor->conaffinity = 1 | 2 | 4;
        floor->margin      = kGeomMargin;
        floor->solref[0]   = kSolref[0];
        floor->solref[1]   = kSolref[1];
        for (int i = 0; i < 5; i++) floor->solimp[i] = (float)kSolimp[i];
    }

    // --- Lights ---
    {
        auto addLight = [&](double dx, double dy, double dz,
                            float diff, float spec_v, float amb) {
            mjsLight* L = mjs_addLight(world_body, nullptr);
            L->type     = mjLIGHT_DIRECTIONAL;
            L->active   = 1;
            L->dir[0] = dx; L->dir[1] = dy; L->dir[2] = dz;
            L->castshadow = 1;
            L->diffuse[0] = diff;   L->diffuse[1] = diff;   L->diffuse[2] = diff;
            L->specular[0] = spec_v; L->specular[1] = spec_v; L->specular[2] = spec_v;
            L->ambient[0] = amb;    L->ambient[1] = amb;    L->ambient[2] = amb;
        };
        addLight(-0.3, -0.2, -1.0, 1.0f, 0.6f, 0.06f);
        addLight( 0.3,  0.2, -1.0, 0.8f, 0.4f, 0.04f);
    }

    double half = world.HalfSize();
    double gap  = world.Gap();

    // --- Build body hierarchy ---
    // Maps: grid pos → spec body name / body pointer (first body at each cell wins)
    std::unordered_map<IVec3, std::string, IVec3Hash>  body_name_map;
    std::unordered_map<IVec3, mjsBody*,    IVec3Hash>  body_ptr_map;
    // Tracks which chain_id first claimed each grid position.
    // Used to distinguish self-intersection (same chain loops back → WELD)
    // from cross-chain junction (different chain passes through → CONNECT, no dup body).
    std::unordered_map<IVec3, int, IVec3Hash>           body_chain_id_map;

    // Body records for post-compile chain-info lookup
    struct BodyRecord {
        std::string name;
        int chain_id;
        int chain_pos;
    };
    std::vector<BodyRecord> body_records;
    body_records.push_back({"world", -1, -1});  // body 0 = worldbody

    // Helper: add geom to body with chain colour.
    // TWO geoms per body:
    //   (1) Visual box: chain colour, NO collision (contype/conaffinity=0).
    //       Renders the familiar cube shape for the user.
    //   (2) Contact sphere: INVISIBLE, radius=half, single contact point.
    //       Sphere always generates exactly 1 contact vs 4 corner points for
    //       a box face — reduces floor contacts from 160 to 40 for a 40-block
    //       chain, cutting constraint count 4x and speeding up CG solver 2-3x.
    auto AddBlockGeom = [&](mjsBody* body, const Chain& chain) {
        // --- Visual box (no collision) ---
        mjsGeom* g = mjs_addGeom(body, nullptr);
        g->type        = mjGEOM_BOX;
        g->size[0]     = g->size[1] = g->size[2] = half;
        g->contype     = 0;   // disabled: no physical contacts on the visual box
        g->conaffinity = 0;
        g->rgba[0]     = chain.color[0]; g->rgba[1] = chain.color[1];
        g->rgba[2]     = chain.color[2]; g->rgba[3] = chain.color[3];

        // --- Contact sphere (invisible, single contact point) ---
        mjsGeom* s = mjs_addGeom(body, nullptr);
        s->type        = mjGEOM_SPHERE;
        s->size[0]     = half;    // radius = half-size → same resting height as box
        s->contype     = 1;
        s->conaffinity = 1;
        s->condim      = 1;       // frictionless (structural skeleton, not locomotion)
        s->rgba[0]     = s->rgba[1] = s->rgba[2] = 0.0f;
        s->rgba[3]     = 0.0f;    // fully transparent
        s->margin      = kGeomMargin;
        for (int i = 0; i < 2; i++) s->solref[i] = kSolref[i];
        for (int i = 0; i < 5; i++) s->solimp[i] = (float)kSolimp[i];
    };

    // Helper: add ball joint with anchor + cone limits
    auto AddBallJoint = [&](mjsBody* body, const IVec3& delta) {
        mjsJoint* j   = mjs_addJoint(body, nullptr);
        j->type       = mjJNT_BALL;
        j->damping[0] = kJointDamping;
        int axis = -1, sign = 1;
        for (int ax = 0; ax < 3; ax++) {
            if ((&delta.x)[ax] != 0) { axis = ax; sign = (&delta.x)[ax]; break; }
        }
        if (axis >= 0) {
            j->pos[0] = j->pos[1] = j->pos[2] = 0.0;
            j->pos[axis] = -sign * (half + gap * 0.5);
        }
        j->limited  = mjLIMITED_TRUE;
        j->range[0] = 0.0;
        j->range[1] = ComputeConeLimit(half, gap) * 0.5;
    };

    // Helper: add WELD equality constraint between two named bodies.
    // Used for SELF-INTERSECTIONS: same chain loops back to a cell it already visited.
    // Both bodies are at the same world position — WELD closes the kinematic loop.
    auto AddWeldConstraint = [&](const std::string& body1_name,
                                 const std::string& body2_name) {
        mjsEquality* eq = mjs_addEquality(spec, nullptr);
        eq->type    = mjEQ_WELD;
        eq->objtype = mjOBJ_BODY;
        mjs_setString(eq->name1, body1_name.c_str());
        mjs_setString(eq->name2, body2_name.c_str());
        eq->data[0] = eq->data[1] = eq->data[2] = 0.0;
        eq->data[3] = eq->data[4] = eq->data[5] = 0.0;
        eq->data[6] = 1.0;  // qw = 1 → identity rotation
        eq->data[7] = eq->data[8] = eq->data[9] = 0.0;
    };

    // Helper: add CONNECT equality constraint between two named bodies.
    // Used for CROSS-CHAIN JUNCTIONS: chain A passes through a cell already
    // occupied by chain B.  Instead of a duplicate body, we connect the last
    // body of chain A (approaching the junction) to the existing junction body
    // with a ball-and-socket constraint (3 positional DOF, no rotational fix).
    // Anchors are placed at the midpoint of the gap between the two faces so
    // the constraint has zero residual in the rest pose.
    auto AddConnectConstraint = [&](const std::string& b1_name,
                                    const std::string& b2_name,
                                    const IVec3& delta) {
        mjsEquality* eq = mjs_addEquality(spec, nullptr);
        eq->type    = mjEQ_CONNECT;
        eq->objtype = mjOBJ_BODY;
        mjs_setString(eq->name1, b1_name.c_str());
        mjs_setString(eq->name2, b2_name.c_str());
        // Zero all data first — mjs_defaultEquality sets data[1]=1 by default.
        for (int i = 0; i < 11; i++) eq->data[i] = 0.0;
        int axis = -1, sign = 1;
        for (int ax = 0; ax < 3; ax++) {
            if ((&delta.x)[ax] != 0) { axis = ax; sign = (&delta.x)[ax]; break; }
        }
        if (axis >= 0) {
            double anchor = half + gap * 0.5;  // midpoint of gap, in each body's frame
            eq->data[axis]     =  sign * anchor;  // anchor1: face of b1 toward b2
            eq->data[3 + axis] = -sign * anchor;  // anchor2: face of b2 toward b1
        }
    };

    // Process each chain
    for (int ci = 0; ci < (int)world.chains.size(); ci++) {
        const Chain& chain = world.chains[ci];
        if (chain.Empty()) continue;

        // Detect secondary chain: first block already owned by an earlier chain
        bool is_secondary = (ci > 0) && body_ptr_map.count(chain.blocks[0]);
        if (is_secondary && chain.Size() < 2) continue;

        mjsBody* prev_body;
        std::string prev_body_name;
        int      start_idx;

        if (!is_secondary) {
            // ── Primary chain root ──────────────────────────────────────────
            IVec3 b0 = chain.blocks[0];
            double wpos[3];
            world.GridToWorld(b0, wpos);

            mjsBody* root = mjs_addBody(world_body, nullptr);
            std::string rname = "body_c" + std::to_string(ci) + "_0";
            mjs_setName(root->element, rname.c_str());
            root->pos[0] = wpos[0]; root->pos[1] = wpos[1]; root->pos[2] = wpos[2];

            AddBlockGeom(root, chain);

            // Chain-0 root: free joint makes the whole structure float.
            // When root_fixed=true (default), no joint is added — the root body
            // is welded to worldbody, anchoring the entire structure in place.
            // When root_fixed=false, the standard free joint is used.
            if (!world.root_fixed || ci != 0) {
                mjsJoint* fj   = mjs_addJoint(root, nullptr);
                fj->type       = mjJNT_FREE;
                fj->damping[0] = kJointDamping;
            }

            body_name_map[b0]     = rname;
            body_ptr_map[b0]      = root;
            body_chain_id_map[b0] = ci;
            body_records.push_back({rname, ci, 0});
            prev_body      = root;
            prev_body_name = rname;
            start_idx = 1;

        } else {
            // ── Secondary chain root ─────────────────────────────────────────
            // Attach first extension block (blocks[1]) as a child of the
            // junction body.  No free joint, no equality constraint.
            IVec3 junction = chain.blocks[0];
            IVec3 b1       = chain.blocks[1];
            IVec3 delta    = b1 - junction;

            mjsBody* junc_body = body_ptr_map[junction];
            mjsBody* root      = mjs_addBody(junc_body, nullptr);
            std::string rname  = "body_c" + std::to_string(ci) + "_1";
            mjs_setName(root->element, rname.c_str());

            // Position relative to junction body
            root->pos[0] = delta.x * world.CellStride();
            root->pos[1] = delta.y * world.CellStride();
            root->pos[2] = delta.z * world.CellStride();

            AddBlockGeom(root, chain);
            // Skip ball joint when WELD_FULL and neither block is a turn.
            // Without a joint the body is rigidly welded to the parent in the
            // kinematic tree — no equality constraint needed, and nv is reduced.
            // The junction (blocks[0]) is always non-turn (§21.6 invariant).
            {
                bool b1_is_turn = world.grid.count(b1) && world.grid.at(b1).is_turn;
                if (world.weld_level == WELD_NONE || b1_is_turn)
                    AddBallJoint(root, delta);
            }

            if (!body_name_map.count(b1))     body_name_map[b1]     = rname;
            if (!body_ptr_map.count(b1))      body_ptr_map[b1]      = root;
            if (!body_chain_id_map.count(b1)) body_chain_id_map[b1] = ci;
            body_records.push_back({rname, ci, 1});
            prev_body      = root;
            prev_body_name = rname;
            start_idx = 2;
        }

        // ── Remaining blocks ────────────────────────────────────────────────
        for (int bi = start_idx; bi < chain.Size(); bi++) {
            IVec3 delta   = chain.blocks[bi] - chain.blocks[bi - 1];
            IVec3 cur_pos = chain.blocks[bi];

            bool cell_occupied = body_ptr_map.count(cur_pos) > 0;
            std::string existing_name = cell_occupied ? body_name_map[cur_pos] : "";

            // Distinguish cross-chain junction from same-chain self-intersection.
            bool is_self_intersection = cell_occupied &&
                                        body_chain_id_map.count(cur_pos) &&
                                        (body_chain_id_map.at(cur_pos) == ci);

            if (cell_occupied && !is_self_intersection && !existing_name.empty()) {
                // ── Cross-chain junction ────────────────────────────────────
                // The current cell belongs to a DIFFERENT chain.  Instead of
                // creating a duplicate body (which causes visual overlap), we:
                //  1. Add an mjEQ_CONNECT ball-and-socket constraint linking
                //     prev_body to the junction body.
                //  2. Make the junction body the new prev_body so subsequent
                //     blocks in this chain become its children (no visual dup).
                AddConnectConstraint(prev_body_name, existing_name, delta);
                prev_body      = body_ptr_map[cur_pos];
                prev_body_name = existing_name;
                continue;
            }

            // ── Create new body (normal or self-intersection duplicate) ─────
            mjsBody* body = mjs_addBody(prev_body, nullptr);
            std::string bname = "body_c" + std::to_string(ci) + "_" + std::to_string(bi);
            mjs_setName(body->element, bname.c_str());
            body->pos[0] = delta.x * world.CellStride();
            body->pos[1] = delta.y * world.CellStride();
            body->pos[2] = delta.z * world.CellStride();

            AddBlockGeom(body, chain);

            // Skip ball joint when WELD_FULL, neither block is a turn, and this is
            // not a self-intersection.  Self-intersections always keep their ball
            // joint so the weld loop-closure constraint has DOF to close against.
            {
                IVec3 prev_pos   = chain.blocks[bi - 1];
                bool prev_is_turn = world.grid.count(prev_pos) && world.grid.at(prev_pos).is_turn;
                bool cur_is_turn  = world.grid.count(cur_pos)  && world.grid.at(cur_pos).is_turn;
                bool skip = (world.weld_level == WELD_FULL) &&
                            !is_self_intersection &&
                            !prev_is_turn && !cur_is_turn;
                if (!skip) AddBallJoint(body, delta);
            }

            // First body wins at each grid cell
            if (!body_name_map.count(cur_pos))     body_name_map[cur_pos]     = bname;
            if (!body_ptr_map.count(cur_pos))      body_ptr_map[cur_pos]      = body;
            if (!body_chain_id_map.count(cur_pos)) body_chain_id_map[cur_pos] = ci;
            body_records.push_back({bname, ci, bi});
            prev_body      = body;
            prev_body_name = bname;

            // Self-intersection: WELD closes the kinematic loop
            if (is_self_intersection && !existing_name.empty()) {
                AddWeldConstraint(existing_name, bname);
            }
        }
    }

    // --- Exclude close neighbours (grandparent-grandchild pairs) ---
    for (int ci = 0; ci < (int)world.chains.size(); ci++) {
        const Chain& chain = world.chains[ci];
        for (int i = 0; i + 2 < chain.Size(); i++) {
            auto it0 = body_name_map.find(chain.blocks[i]);
            auto it2 = body_name_map.find(chain.blocks[i + 2]);
            if (it0 == body_name_map.end() || it2 == body_name_map.end()) continue;

            mjsExclude* ex = mjs_addExclude(spec);
            mjs_setString(ex->bodyname1, it0->second.c_str());
            mjs_setString(ex->bodyname2, it2->second.c_str());
        }
    }

    // --- Compile ---
    mjModel* model = mj_compile(spec, nullptr);
    if (!model) {
        const char* err = mjs_getError(spec);
        res.error = err ? err : "(compile failed, no error message)";
        return res;
    }
    res.model = model;
    res.data  = mj_makeData(model);

    // --- Build body_chain_info lookup (size = nbody) ---
    g_body_chain_info.assign(model->nbody, {-1, -1});
    for (int ri = 0; ri < (int)body_records.size(); ri++) {
        const BodyRecord& br = body_records[ri];
        int bid = mj_name2id(model, mjOBJ_BODY, br.name.c_str());
        if (bid >= 0 && bid < model->nbody)
            g_body_chain_info[bid] = {br.chain_id, br.chain_pos};
    }
    res.body_chain_info = g_body_chain_info;

    std::cout << "Compiled: nbody=" << model->nbody
              << " nv="   << model->nv
              << " ngeom=" << model->ngeom << "\n";
    return res;
}
