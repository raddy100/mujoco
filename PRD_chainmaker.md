# ChainMaker V2 — Product Requirements Document

## 1. Executive Summary

ChainMaker is a 3D block-placement GUI for designing "beads-on-a-string" robot structures. 
Users place cube-shaped beads along chains in a 3D integer grid. Chains can turn (90° only in 
V1), self-intersect by sharing blocks, and branch from any existing block. The tool has two 
stages: a **Build Stage** (fast visual placement, no physics) and a **Simulation Stage** (MuJoCo 
physics on the compiled structure).

This is V2, replacing the existing `progui/` prototype. The key improvements are:
- **Multi-chain support** with junction blocks
- **No physics during building** — pure grid data structure + visualization-only rendering
- **Instant block placement** — no `mj_recompile()` per block
- **Scalable** — support hundreds or thousands of blocks without performance degradation

---

## 2. Core Concepts

### 2.1 The Grid

The world is an integer-indexed 3D grid. Each cell at position `(ix, iy, iz)` can be:
- **Empty** — no block
- **Occupied** — contains a block (cube or turn bead)

The physical position of grid cell `(ix, iy, iz)` in world coordinates is:
```
world_x = ix * (bead_size + gap)
world_y = iy * (bead_size + gap)  
world_z = iz * (bead_size + gap) + z_offset
```
where `bead_size` is the uniform cube edge length, `gap` is the spacing between cubes, and 
`z_offset` lifts the grid off the floor.

### 2.2 Blocks

A **block** occupies exactly one grid cell. Two types:

| Type | Shape (V1) | Axis Usage | Intersection Rule |
|------|-----------|------------|-------------------|
| **Straight** | Cube | 1 axis (chain enters one face, exits opposite) | Up to 3 chains (one per axis: X, Y, Z) |
| **Turn** | Cube (V1), right-triangle wedge (future) | 2 axes (entry axis ≠ exit axis) | Exclusive to 1 chain, no intersection |

### 2.3 Chains

A **chain** is an ordered sequence of grid cells representing a path of beads on a string. 
Properties:
- Each chain has a unique ID and display color
- A chain has a **head** (the last placed block, where new blocks are added)
- A chain has a **direction** (the axis along which the next block will be placed)
- Chains cannot reverse direction (no 180° U-turns)
- Consecutive blocks in a chain are always grid-adjacent (Manhattan distance = 1)

### 2.4 Junctions

When a new chain starts from an existing block, that block becomes a **junction**. The junction 
block is shared between chains. The new chain claims an available axis on the junction block and 
extends outward.

A straight block at a junction can participate in up to 3 chains (X, Y, Z axes). A turn block 
cannot be a junction (it uses 2 axes and is exclusive).

---

## 3. Data Model

### 3.1 GridCell

```cpp
struct GridCell {
    glm::ivec3 pos;          // integer grid coordinates (ix, iy, iz)
    
    // Axis occupancy: which chains pass through this cell on each axis
    // -1 = unoccupied, >= 0 = chain ID
    int chain_on_axis[3];    // [X_axis, Y_axis, Z_axis]
    
    bool is_turn;            // true if this is a turn block
    int turn_entry_axis;     // for turn blocks: which axis the chain enters on
    int turn_exit_axis;      // for turn blocks: which axis the chain exits on
};
```

### 3.2 Chain

```cpp
struct Chain {
    int id;                            // unique chain ID
    std::string name;                  // display name (e.g., "Chain A")
    float color[4];                    // RGBA display color
    std::vector<glm::ivec3> blocks;    // ordered sequence of grid positions
    int head_direction;                // current spawn face (SpawnFace enum)
    bool is_active;                    // true if this is the chain being edited
};
```

### 3.3 World

```cpp
struct ChainWorld {
    // Grid storage: maps grid position → cell data
    std::unordered_map<glm::ivec3, GridCell> grid;
    
    // All chains
    std::vector<Chain> chains;
    int active_chain_id;               // which chain is currently being extended
    
    // Global parameters
    double bead_size;                  // cube edge length (meters), range [0.01, 0.10]
    double gap_ratio;                  // gap as fraction of bead_size
    
    // Derived
    double cell_stride() const { return bead_size + gap_ratio * bead_size; }
};
```

### 3.4 Direction / SpawnFace (carried from progui)

```cpp
enum SpawnFace {
    FACE_POSX = 0,  // +X direction
    FACE_NEGX = 1,  // -X direction
    FACE_POSY = 2,  // +Y direction
    FACE_NEGY = 3,  // -Y direction
    FACE_POSZ = 4,  // +Z direction
    FACE_NEGZ = 5,  // -Z direction
};
```

---

## 4. Build Stage — Detailed Behavior

### 4.1 Rendering Approach

During the Build Stage, **MuJoCo physics is not used at all**. Instead:

1. Create an `mjvScene` with sufficient capacity (e.g., `maxgeom = 10000`)
2. Each frame, reset `scn.ngeom = 0`
3. For each occupied grid cell, call `mjv_initGeom()` to add a visual-only box:
   - Type: `mjGEOM_BOX` (solid) or `mjGEOM_LINEBOX` (wireframe for ghost/preview)
   - Size: `bead_size / 2` (half-extents)
   - Position: computed from grid coordinates × cell_stride
   - Color: chain color (with alpha for transparency effects)
4. Add a ghost/preview block at the next placement position (semi-transparent)
5. Add floor plane geom
6. Add axis indicators / grid lines as needed
7. Render with `mjr_render(viewport, &scn, &con)`

This approach uses MuJoCo's OpenGL renderer for nice visuals but avoids any physics 
compilation. Block placement is O(1) — just add an entry to the grid map.

### 4.2 Camera and Navigation

Reuse MuJoCo's camera system:
- `mjv_moveCamera()` for orbit, pan, zoom via mouse
- `mjvCamera` with free camera mode
- Same mouse controls as progui (left-drag rotate, right-drag pan, scroll zoom)

### 4.3 Block Placement Flow

**Extending the active chain:**
1. Ghost preview shows where next block will go (semi-transparent, chain color)
2. Press `C` (or click "Place Block") to place a block at the ghost position
3. The ghost advances to the next position along the current direction
4. If the ghost position is occupied by a compatible block (same chain can share the axis), 
   the chain threads through it and the ghost advances past it

**Changing direction (turning):**
1. Arrow keys (←→↑↓) and `5`/`6` change the spawn direction (same as progui)
2. When the direction changes, the current head block is marked as a **turn block**
3. The turn block's `turn_entry_axis` and `turn_exit_axis` are recorded
4. If the current head block is already a junction (shared with another chain), the turn is 
   **rejected** (turn blocks cannot be shared)

**Validation rules before placing a block:**
- Target cell is within world bounds (if we impose bounds)
- Target cell is empty, OR target cell is a straight block with the current axis unoccupied
- If the current head would become a turn block, it must not be a junction
- Chain is not reversing direction (not going back the way it came)

### 4.4 Multi-Chain Workflow

**Starting a new chain:**
1. Click "New Chain" button → enter chain creation mode
2. Click on any existing block in the 3D view (mouse picking via `mjv_select` or custom 
   ray-cast against grid)
3. The selected block becomes the junction. A direction picker appears (showing available 
   axes on that block)
4. Select a direction → new chain is created with that block as its first entry, extending 
   in the chosen direction
5. The new chain becomes the active chain

**Switching between chains:**
- Dropdown or numbered buttons to select the active chain
- Keyboard shortcuts (e.g., `1`-`9` for quick chain selection)
- The active chain's head and ghost are highlighted
- Non-active chains are rendered slightly dimmer

### 4.5 Block Deletion

- `Delete` key removes the last block from the active chain (undo last placement)
- If the deleted block was a junction, the other chains' references remain (the block still 
  exists for those chains)
- If the deleted block is only in the active chain and no other chains reference it, the grid 
  cell becomes empty

### 4.6 Save / Load

**Save format:** JSON file containing:
- Global parameters (bead_size, gap_ratio)
- List of chains, each with:
  - Chain ID, name, color
  - Ordered list of grid positions `[ix, iy, iz]`
  - Block type per position (straight or turn, with entry/exit axes)

The format must be **machine-readable** for export to AI systems and external programs. 
JSON is the primary format. Additional exporters (e.g., CSV, YAML, or custom DSL) can be 
added later. The old progui direction-token format is **not** supported — progui is reference 
only.

**Load:** Reconstructs the grid and chains from the saved JSON data.

---

## 5. Simulation Stage — Detailed Design

### 5.1 Compilation Pipeline

When the user clicks "Start Simulation":

1. **Create `mjSpec`** via `mj_makeSpec()`
2. **Configure compiler options:**
   - `spec->opt.jacobian = mjJAC_AUTO` (auto-selects sparse when `nv ≥ 60`)
   - `spec->opt.solver = mjSOL_NEWTON` (Newton solver — best for chain structures)
   - `spec->opt.integrator = mjINT_IMPLICITFAST` (stable + fast implicit integration)
   - `spec->opt.timestep = 0.002` (2ms default, tunable)
   - `spec->opt.noslip = 1` (prevent drift at contacts)
3. **Add environment:** floor geom, lights, skybox
4. **Build nested body hierarchy per chain:**

   The string connecting beads is modeled by the **kinematic tree itself**. Each chain 
   becomes a nested body hierarchy where each bead is a child of the previous bead, 
   connected by ball joints. MuJoCo's generalized coordinate system guarantees that 
   parent-child bodies **cannot separate** — the "string" connectivity is a hard 
   mathematical invariant, not a soft constraint.

   **For each chain (primary chain first):**
   - The first bead of the first chain gets a `freejoint` (6 DOF — anchors chain to world)
   - Each subsequent bead is created as a **child body** of the previous bead:
     ```
     parent_body = previous bead's body
     child_body = mjs_addBody(parent_body, NULL)
     child_body->pos = [0, 0, bead_size + gap]  // offset along chain direction
     ```
   - Add `mjs_addGeom(child_body, NULL)` → `mjGEOM_BOX` with half-extents = `bead_size / 2`
   - Set geom `contype = 1`, `conaffinity = 1` (default collision group)
   - Add `mjs_addJoint(child_body, NULL)` → ball joint (`mjJNT_BALL`):
     - `joint->type = mjJNT_BALL`
     - `joint->limited = 1`
     - `joint->range[1] = atan2(gap, bead_size) * (180/π)` degrees (cone limit)
     - `joint->pos` = anchor point at the face center between parent and child
   - Joint damping: `joint->damping = kJointDamping` (resists angular velocity)

5. **Why ball joints model the string:**
   - A ball joint provides **3 rotational DOFs** — the bead can tilt/twist relative 
     to its neighbor, exactly like a bead pivoting on a string
   - **Zero translational DOF** — the parent-child offset is fixed by the kinematic 
     tree, so beads cannot separate (string cannot stretch)
   - **Joint angle limits** prevent adjacent beads from interpenetrating — the cone 
     limit restricts how far a bead can tilt before hitting its neighbor
   - The result is physically accurate: rigid beads that can rotate freely at their 
     contact points but cannot pull apart

6. **For junction blocks (multi-chain branching):**
   - The junction body already exists in the primary chain's hierarchy
   - Secondary chains branching from a junction use `mjEQ_CONNECT` equality constraints:
     ```
     eq->type = mjEQ_CONNECT
     eq->name1 = junction_body_name
     eq->name2 = secondary_chain_first_bead_name
     eq->data[0..2] = anchor point on junction body face
     ```
   - The secondary chain is its own nested hierarchy (bead → child bead → child bead...)
     starting from the junction's first extension bead. This first extension bead is a 
     **root body under worldbody** with its own `freejoint` (6 DOF). The `mjEQ_CONNECT` 
     constraint pins it to the junction body's face, while the freejoint allows the solver 
     to find the equilibrium position.
   - `mjEQ_CONNECT` is a soft constraint (solver-enforced) — use stiff `solref` values 
     (e.g., `[0.005, 1.0]`) to minimize separation at junctions

7. **Configure collision filtering** (see §5.3)
8. **Compile once:** `mjModel* m = mj_compile(spec, NULL)`
9. **Create data:** `mjData* d = mj_makeData(m)`
10. **Cache compiled model:** `mj_saveModel(m, "chainmaker_cache.mjb", NULL, 0)` for fast reload
11. Switch UI to simulation mode

> **Design rationale — joints vs tendons:** MuJoCo tendons are 1D scalar elements that 
> measure path length through sites and can apply spring/damper/limit forces. They are 
> useful for cable-driven mechanisms but are **not needed** for structural chain modeling. 
> The nested body hierarchy with ball joints already provides: (a) rigid connectivity 
> (cannot stretch — hard kinematic constraint), (b) rotational freedom at each link 
> (3 DOF per ball joint), and (c) angular limits preventing interpenetration. Tendons 
> would add a redundant global length constraint at additional computational cost. 
> Tendons will be used in **Phase 3** for actuator "muscles" — cables routed through 
> the structure that pull to create movement.

### 5.2 Return to Build Mode

- "Stop Simulation" → `mj_deleteData(d)`, `mj_deleteModel(m)`, return to grid view
- The `ChainWorld` grid data is the **source of truth**; the MuJoCo model is ephemeral
- Optionally save the simulation state as a keyframe before returning

### 5.3 Collision Filtering Strategy

Chain-based robots have a critical collision problem: adjacent blocks on the same chain will 
always be in near-contact (they're beads on a string). We need adjacent blocks to NOT collide 
with each other, but distant blocks on the same chain SHOULD collide (self-intersection creates 
rigidity).

**Multi-layer filtering approach:**

1. **Parent-child auto-filter (free):** MuJoCo automatically disables collisions between 
   parent and child bodies. Since each chain uses a nested hierarchy (each bead is a child 
   of the previous bead), **adjacent blocks automatically don't collide**. This is the 
   primary filtering mechanism and costs nothing.

2. **Exclude pairs for near-adjacent blocks:** The parent-child filter only covers immediate 
   neighbors. For blocks 2 steps apart (grandparent-grandchild), add explicit excludes:
   ```
   mjs_addExclude(spec, body_name_A, body_name_C)  // skip 1 block apart
   ```
   For a chain of N blocks, this adds ~N-2 exclude pairs. Combined with parent-child 
   auto-filter, blocks within 2 steps of each other never collide.

3. **Custom contact filter for chain-distance logic:** Install `mjcb_contactfilter` callback:
   ```c
   int chainmaker_contact_filter(const mjModel* m, mjData* d, int geom1, int geom2) {
       int body1 = m->geom_bodyid[geom1];
       int body2 = m->geom_bodyid[geom2];
       int chain_dist = get_chain_distance(body1, body2);
       if (chain_dist >= 0 && chain_dist < MIN_COLLISION_DISTANCE) {
           return 1;  // filter OUT (don't collide)
       }
       return 0;  // allow collision
   }
   ```
   Where `get_chain_distance()` returns the number of blocks between two bodies along 
   any shared chain, or -1 if they're on different chains (always allow cross-chain collision).
   `MIN_COLLISION_DISTANCE` = 3 (tunable) — blocks within 3 steps don't collide.

4. **Contype/conaffinity bitmask groups (future):** If needed, assign different chain segments 
   to different collision groups for coarser filtering. The 32-bit bitmask supports up to 32 
   independent groups.

**Performance characteristics:**
- Broadphase (Sweep-and-Prune): O(n log n) — scales well to 1000+ bodies
- Exclude list: binary search O(log E) per candidate pair
- Custom callback: O(1) per candidate pair (lookup table indexed by body ID)
- Narrowphase: only runs on pairs that pass all filters

### 5.4 Performance Monitoring & Profiling

MuJoCo has 15 built-in hierarchical timers (`mjTIMER_STEP` through `mjTIMER_COL_NARROW`) 
that are **zero-cost when disabled** and activate only when `mjcb_time` is installed. 
ChainMaker will integrate comprehensive profiling from day one.

**Timer hierarchy (what we monitor):**
```
mjTIMER_STEP                         ← Total step time
├── mjTIMER_POSITION                 ← Position-dependent computations
│   ├── mjTIMER_POS_KINEMATICS       ← Joint transforms, COM, tendons
│   ├── mjTIMER_POS_INERTIA          ← Mass matrix build + LDL factorization
│   ├── mjTIMER_POS_COLLISION        ← Total collision time
│   │   ├── mjTIMER_COL_BROAD        ← Broadphase (SAP algorithm)
│   │   └── mjTIMER_COL_NARROW       ← Narrowphase (geometry tests)
│   ├── mjTIMER_POS_MAKE             ← Build constraint Jacobians
│   └── mjTIMER_POS_PROJECT          ← Project constraints
├── mjTIMER_VELOCITY                 ← Velocity-dependent computations
├── mjTIMER_ACTUATION                ← Actuator force computation
├── mjTIMER_CONSTRAINT               ← Constraint solver (PGS/CG/Newton)
└── mjTIMER_ADVANCE                  ← Time integration (Euler/RK4/Implicit)
```

**Implementation:**

1. **Install timer callback:**
   ```c
   static mjtNum chainmaker_timer() {
       static auto start = std::chrono::steady_clock::now();
       auto elapsed = std::chrono::duration<double, std::milli>(
           std::chrono::steady_clock::now() - start);
       return elapsed.count();
   }
   mjcb_time = chainmaker_timer;
   ```

2. **Per-step profiling readout:**
   ```c
   void print_profile(const mjData* d) {
       for (int i = 0; i < mjNTIMER; i++) {
           if (d->timer[i].number > 0) {
               double avg_ms = d->timer[i].duration / d->timer[i].number;
               printf("%s: %.3f ms/step (%d calls)\n",
                      mjTIMERSTRING[i], avg_ms, d->timer[i].number);
           }
       }
   }
   ```

3. **Real-time profiler overlay in simulation mode:**
   - Display stacked bar chart in the GUI (same pattern as simulate app's profiler)
   - Show: Total step time, Collision %, Solver %, Inertia %, Other %
   - Warning indicators when step time exceeds real-time threshold

4. **Memory utilization monitoring:**
   - `d->maxuse_arena` — peak arena usage (contacts + constraints)
   - `d->maxuse_stack` — peak stack usage (temporary computations)
   - `d->maxuse_con` — peak contact count
   - `d->maxuse_efc` — peak constraint count
   - Display these in the profiler panel

5. **Warning counters:**
   - `mjWARN_CONTACTFULL` — contact buffer overflow (increase `nconmax`)
   - `mjWARN_CNSTRFULL` — constraint buffer overflow (increase `njmax`)
   - `mjWARN_VGEOMFULL` — visual geom overflow (increase scene capacity)
   - Display active warnings prominently in UI

6. **Profiling log export:** Save timer data to CSV for offline analysis:
   ```
   step_number, total_ms, collision_ms, solver_ms, inertia_ms, contacts, constraints
   ```

### 5.5 Performance Budget & Bottleneck Analysis (1000+ Blocks)

For a structure with 1000 blocks, here are the expected scaling characteristics:

| Component | Scaling | Estimated @ 1000 blocks | Bottleneck Risk |
|-----------|---------|------------------------|-----------------|
| Broadphase (SAP) | O(n log n) | ~0.5–1 ms | Low |
| Narrowphase | O(n_contacts) | ~1–5 ms (depends on self-contact) | Medium |
| Mass matrix build | O(n) sparse | ~0.5 ms | Low |
| Mass matrix factor (LDL) | O(n) for tree topology | ~0.5 ms | Low |
| Constraint Jacobian | O(n_constraints × chain_length) | ~1–3 ms | Medium |
| Constraint solver | O(iterations × n_constraints) | ~2–10 ms | **High** |
| Integration | O(n) | ~0.2 ms | Low |
| **Total step** | — | ~5–20 ms | — |

**Key insight:** At `nv ≥ 60` DOF (which 1000 blocks certainly exceeds), MuJoCo auto-switches 
to **sparse Jacobians** (`mjJAC_AUTO`). The mass matrix is always stored sparse (CSR format). 
The crossover point is `nv = 60` — our models will have `nv ≈ 3000–3100` (3 DOF per ball joint × 
~999 internal beads + 6 DOF per chain root freejoint × ~number_of_chains), so sparse mode is 
mandatory.

**Mitigation strategies for bottlenecks:**

1. **Constraint solver tuning:**
   - Use Newton solver (`mjSOL_NEWTON`) — quadratic convergence, fewer iterations
   - Reduce `opt.iterations` (default 100) to minimum acceptable (try 20–50)
   - Enable island decomposition for independent sub-chains
   - Set `opt.tolerance` appropriately (1e-8 default, try 1e-6 for faster convergence)

2. **Contact reduction:**
   - Aggressive collision filtering (§5.3) reduces narrowphase and solver load
   - Set `nconmax` to expected peak (not unlimited) to bound memory
   - Use `condim = 1` (frictionless) for block-to-block contacts where friction isn't needed

3. **Sparse matrix operations:**
   - Force `opt.jacobian = mjJAC_SPARSE` explicitly for predictability
   - Enable AVX/SIMD (`mjUSEPLATFORMSIMD`) for supernode-accelerated sparse operations

4. **Thread pool:**
   - MuJoCo supports `mjThreadPool` for parallel island solving
   - `mju_threadPoolCreate()` / `mju_threadPoolEnqueue()` for task-parallel constraint solving
   - Each island's Newton solve runs independently on a separate thread

5. **Timestep optimization:**
   - Use `mjINT_IMPLICITFAST` — symmetric positive-definite factorization (LDL, not LU)
   - Larger timestep (e.g., 5ms instead of 2ms) if stability allows → fewer steps/second
   - Semi-implicit Euler (`mjINT_EULER`) is fastest but least stable for chain structures

### 5.6 Video Recording & Headless Simulation

For long simulation runs or parameter sweeps, ChainMaker supports headless rendering:

**Offscreen rendering pipeline:**
1. Create OpenGL context (GLFW invisible window or EGL on Linux)
2. `mjr_makeContext(m, &con, mjFONTSCALE_150)` — creates offscreen framebuffer
3. `mjr_setBuffer(mjFB_OFFSCREEN, &con)` — switch to offscreen rendering
4. For each frame:
   ```c
   mj_step(m, d);
   mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
   mjr_render(viewport, &scn, &con);
   mjr_readPixels(rgb_buffer, depth_buffer, viewport, &con);
   // flip vertically (OpenGL origin is bottom-left)
   // write frame to video/image sequence
   ```
5. Output options:
   - Raw RGB frames → pipe to ffmpeg for video encoding
   - PNG sequence via lodepng (already a MuJoCo dependency)
   - Reference implementation: `sample/record.cc`

**Use cases:**
- Record simulation runs for review without real-time constraints
- Generate training data for ML-based robot design optimization
- Parameter sweep: vary bead_size, gap_ratio, chain topology → batch simulate → compare results
- Share simulation results as video files

---

## 6. Actuator Architecture — Linear Actuators as "Muscles"

*(Phase 3 — implementation after Build Stage GUI is complete)*

The chain structures are purely skeletal (bones). Linear actuators provide the "muscles" that 
create movement. Two actuator types are planned:

### 6.1 Supported Actuator Types

| Actuator | Physical Device | MuJoCo Mapping | Use Case |
|----------|----------------|----------------|----------|
| **Push/Pull Solenoid** | Binary on/off, fixed stroke | Slide joint + position actuator with `ctrlrange=[0, stroke]` | Quick binary movements |
| **Ball Screw Linear Motor** | Continuous position, high force | Slide joint + general actuator with `mjDYN_FILTER` dynamics | Precise positioning |

### 6.2 MuJoCo Implementation

Each linear actuator maps to:

1. **Slide joint** (`mjJNT_SLIDE`) on the target body along the actuator axis
   - `range = [min_extension, max_extension]` (stroke limits)
   - `stiffness`, `damping` for passive dynamics

2. **Actuator** via `mjs_addActuator(spec)`:
   - **Transmission:** `mjTRN_JOINT` — applies force directly on the slide joint
   - **Gain:** `mjGAIN_FIXED` (constant gain) or `mjGAIN_AFFINE` (position-dependent)
   - **Bias:** `mjBIAS_AFFINE` — models `[bias_constant, -Kp, -Kv]` for PD control:
     `force = gain * ctrl + bias[0] + bias[1] * position + bias[2] * velocity`
   - **Dynamics:** `mjDYN_FILTER` with `dynprm[0] = time_constant` for motor response lag
   - **Force limits:** `forcerange = [-max_force, max_force]`
   - **Control limits:** `ctrlrange = [min_ctrl, max_ctrl]`

3. **Alternative: Cable-driven actuators via tendons (future consideration):**
   - Spatial tendons (`mjs_addTendon` + `mjs_wrapSite`) can route cables through 
     the chain structure, modeling cable-driven mechanisms
   - Actuator transmission `mjTRN_TENDON` applies force along the tendon path
   - Useful for modeling pull-cables that run through the bead chain
   - Tendons measure path length through sites and can enforce length limits
   - This is where MuJoCo tendons become relevant — as actuator transmission 
     elements, NOT as the structural string (which is modeled by the kinematic tree)

4. **Control callback** (`mjcb_control`):
   ```c
   void chainmaker_control(const mjModel* m, mjData* d) {
       // For each actuator, set d->ctrl[i] based on:
       // - User input (manual control)
       // - Scripted trajectory (automated movement)
       // - PID controller output
   }
   ```

### 6.3 Actuator Monitoring — Sensor Suite

For each linear actuator, attach the following sensors:

| Measurement | MuJoCo Sensor Type | Data Access | Physical Meaning |
|-------------|-------------------|-------------|------------------|
| Position | `mjSENS_JOINTPOS` | `d->sensordata[sid]` | Extension length (meters) |
| Velocity | `mjSENS_JOINTVEL` | `d->sensordata[sid]` | Extension speed (m/s) |
| Force | `mjSENS_ACTUATORFRC` | `d->sensordata[sid]` | Applied force (Newtons) |
| Joint torque | `mjSENS_JOINTACTFRC` | `d->sensordata[sid]` | Reaction force at joint |
| Contact force | `mjSENS_TOUCH` | `d->sensordata[sid]` | Normal force on body |
| Kinetic energy | `mjSENS_E_KINETIC` | `d->sensordata[sid]` | Motion energy (Joules) |
| Potential energy | `mjSENS_E_POTENTIAL` | `d->sensordata[sid]` | Stored energy (Joules) |

**Derived measurements** (computed in ChainMaker, not native MuJoCo):

| Measurement | Derivation | Purpose |
|-------------|-----------|---------|
| Electrical current | `I = F / K_force` (force constant) | Motor current draw estimate |
| Voltage | `V = I * R + K_emf * velocity` | Back-EMF model for voltage |
| Resistance (thermal) | `R(T) = R_0 * (1 + α * ΔT)` | Thermal resistance model |
| Friction | `F_friction = μ * F_normal` | Friction estimation from contact sensors |
| Power | `P = F * v` (force × velocity) | Mechanical power output |
| Efficiency | `P_out / P_in` (mechanical / electrical) | Overall actuator efficiency |

These derived measurements are modeled using custom sensor callbacks (`mjcb_sensor`) or 
computed post-step from the native sensor data. The electrical model parameters (force 
constant, resistance, back-EMF coefficient) are configurable per actuator and stored in the 
JSON save file.

### 6.4 PID Controller Plugin (Reference)

MuJoCo includes a built-in PID actuator plugin (`plugin/actuator/pid.cc`) with:
- Proportional, integral, derivative gains
- Integral wind-up clamping (`imax`)
- Slew rate limiting (`slewmax`)
- Setpoint tracking

ChainMaker can use this plugin directly for position-controlled linear actuators, or implement 
a custom `mjcb_control` callback for more complex control strategies (e.g., force control, 
impedance control, trajectory tracking).

---

## 7. Multi-Robot Scaling Strategy

*(Phase 5 — implementation after single-robot simulation is validated)*

### 7.1 Scale Target

| Parameter | Target |
|-----------|--------|
| Blocks per robot | 1,000 |
| Actuators per robot | 15 |
| Robots in scene | 100 |
| Total bodies | 100,000 |
| Total actuators | 1,500 |
| Total DOF (est.) | ~300,000 (3 DOF/ball joint + 6 DOF/chain root) |

### 7.2 Model Composition with `mjs_attach()`

Each robot is designed as an independent `mjSpec`. To combine multiple robots:

```c
mjSpec* scene = mj_makeSpec();
// Add environment (floor, lights, etc.)

for (int i = 0; i < num_robots; i++) {
    mjSpec* robot = load_robot_spec(robot_file);
    // Create a frame at the robot's spawn position
    mjsBody* base = mjs_findBody(scene, "world");
    mjsFrame* frame = mjs_addFrame(base, NULL);
    frame->pos[0] = spawn_x[i];
    frame->pos[1] = spawn_y[i];
    frame->pos[2] = spawn_z[i];
    
    // Attach with unique prefix to avoid name collisions
    char prefix[32];
    snprintf(prefix, sizeof(prefix), "r%d_", i);
    mjs_attach(frame->element, robot->element, prefix, "");
}

mjModel* m = mj_compile(scene, NULL);
```

**Key considerations:**
- `prefix`/`suffix` parameters namespace all element names (bodies, joints, sensors, actuators)
- All assets from child models are copied into the parent spec
- Circular references are NOT checked — ensure robot specs are independent
- Model compilation time scales with total element count (one-time cost)

### 7.3 Simulation Approaches for 100 Robots

**Approach A: Single combined model (preferred for interaction)**
- All robots in one `mjModel`, one `mjData`, one physics step
- Pros: Robots can physically interact (push, pull, collide)
- Cons: 100K bodies is very large; requires aggressive optimization
- Mitigation: Island decomposition (robots that aren't touching = independent islands)
- Use `mjThreadPool` for parallel island solving

**Approach B: Parallel independent simulations**
- Each robot gets its own `mjModel`/`mjData` pair
- `mjModel` is read-only at runtime — compile once, create 100 `mjData` instances
- Step each `mjData` independently (trivially parallel, no locking)
- Pros: Perfect scaling, each sim runs at full speed
- Cons: No inter-robot physics interaction
- Use case: Parameter sweeps, training data generation, pre-screening designs

**Approach C: MJX (JAX-based GPU acceleration)**
- MuJoCo's JAX port (`mjx/`) can batch thousands of simulations on GPU
- Identical model, different initial conditions → vectorized physics
- Pros: Massive throughput (10,000+ parallel sims)
- Cons: Not all MuJoCo features supported; requires JAX/GPU setup
- Use case: Automated design optimization, reinforcement learning

### 7.4 Rendering at Scale

For 100 robots × 1000 blocks = 100,000 visual geoms:

- `mjv_makeScene(m, &scn, 200000)` — allocate sufficient geom capacity
- Monitor `mjWARN_VGEOMFULL` warning for overflow
- Use LOD (level-of-detail) strategy:
  - Close robots: full detail (individual block rendering)
  - Distant robots: simplified mesh or bounding-box representation
  - Off-screen robots: skip rendering entirely
- Consider instanced rendering (future MuJoCo enhancement) for identical blocks
- Offscreen rendering for batch simulation (no GPU display overhead)

### 7.5 Control Architecture for Multi-Robot

```
┌─────────────────────────────────────────┐
│          Central Controller             │
│  (scripted movements, AI planner)       │
├─────────────────────────────────────────┤
│  mjcb_control callback                  │
│  For each robot i (0..99):              │
│    For each actuator j (0..14):         │
│      d->ctrl[actuator_index(i,j)] =     │
│        compute_control(i, j, d, t)      │
└─────────────────────────────────────────┘
```

- **Manual control:** Select a robot, adjust its actuators via sliders
- **Scripted trajectories:** Define keyframed paths, interpolate control signals
- **Coordinated movement:** Multiple robots following synchronized programs
- **Automated search:** Optimize movement patterns via gradient-free methods or RL

---

## 8. UI Layout

```
┌──────────────────────────────────────┬─────────────────┐
│                                      │  Chain Controls  │
│                                      │  ─────────────── │
│           3D Viewport                │  [Place Block]   │
│                                      │  [Delete Block]  │
│       (rendered with MuJoCo          │  [New Chain]     │
│        visualization API)            │  ─────────────── │
│                                      │  Active: Chain A │
│                                      │  Dir: +X  ►      │
│                                      │  Blocks: 42      │
│                                      │  ─────────────── │
│                                      │  Bead Size: 0.05 │
│                                      │  Gap Ratio: 0.05 │
│                                      │  ─────────────── │
│                                      │  [Save Chain]    │
│                                      │  [Load Chain]    │
│                                      │  ─────────────── │
│                                      │  [Simulate ►]    │
└──────────────────────────────────────┴─────────────────┘
 Status: Chain A | Head: (3, 5, 2) | Direction: +X | 156 total blocks
```

UI is built with MuJoCo's `mjUI` widget system (same as progui and simulate app), rendered on 
the right side of the viewport.

---

## 9. Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `C` | Place block (extend active chain) |
| `Delete` | Delete last block from active chain |
| `←` `→` `↑` `↓` | Change direction (±X, ±Y) |
| `Z` / `X` | Change direction (+Z / -Z) |
| `N` | New chain mode (then click a block) |
| `1`-`9` | Switch active chain |
| `+` / `-` | Increase / decrease gap ratio |
| `P` | Enter simulation mode |
| `R` | Reset to build mode (from simulation) |
| `Ctrl+S` | Save |
| `Ctrl+L` | Load |
| Mouse left-drag | Rotate camera |
| Mouse right-drag | Pan camera |
| Scroll | Zoom |

---

## 10. Technical Architecture

### 10.1 File Structure

```
chainmaker/
├── CMakeLists.txt              # Build config (links mujoco, glfw)
├── main.cc                     # Entry point, GLFW window, render loop
├── chainmaker.h                # Core data structures (GridCell, Chain, ChainWorld)
├── chainmaker.cpp              # Grid logic, chain operations, validation
├── chainmaker_render.h         # Rendering interface
├── chainmaker_render.cpp       # mjvScene population, floor, ghost preview
├── chainmaker_ui.h             # UI panel definitions
├── chainmaker_ui.cpp           # UI button handlers, keyboard, mouse
├── chainmaker_compile.h        # Grid → mjSpec compiler (for simulation stage)
├── chainmaker_compile.cpp      # Compilation logic, collision filtering
├── chainmaker_sim.h            # Simulation stage interface
├── chainmaker_sim.cpp          # Physics stepping, profiling overlay, video recording
├── chainmaker_profiler.h       # Performance monitoring interface
├── chainmaker_profiler.cpp     # Timer readout, memory stats, CSV export
├── chainmaker_actuator.h       # Actuator definitions and control interface
├── chainmaker_actuator.cpp     # Linear actuator setup, sensor suite, PID
├── chainmaker_io.h             # Save/load interface
├── chainmaker_io.cpp           # File I/O (JSON format)
└── chainmaker_multirobot.h/.cpp # Multi-robot composition and control (Phase 5)
```

### 10.2 Rendering Pipeline (Build Stage)

```
Each frame:
  1. Clear scene (scn.ngeom = 0)
  2. Add floor plane geom (mjGEOM_PLANE)
  3. For each GridCell in world.grid:
     a. Compute world position from grid coords
     b. Determine color from chain ownership
     c. mjv_initGeom() → mjGEOM_BOX into scn.geoms[scn.ngeom++]
  4. Add ghost preview block (mjGEOM_BOX, semi-transparent)
  5. Add chain head marker (mjGEOM_SPHERE, small, on active chain head)
  6. Add direction indicator (mjGEOM_ARROW from head in spawn direction)
  7. mjr_render(viewport, &scn, &con)
  8. mjr_overlay() for status text
  9. mjui_render() for UI panel
```

### 10.3 Mouse Picking (Block Selection)

For "New Chain" mode, we need to click on existing blocks. Two approaches:

**Option A: Color-based picking** — Render each block with a unique color to an offscreen 
buffer, read back the pixel under the mouse cursor, decode the grid position.

**Option B: Ray-grid intersection** — Compute a ray from the camera through the mouse 
position, intersect with the axis-aligned grid to find which cell is hit. This is the 
preferred approach since the grid is regular and intersection is trivial:
```
ray = camera_pos + t * ray_dir
For each axis, compute which grid plane the ray hits first, check if that cell is occupied.
```

### 10.4 Compilation Pipeline (Simulation Stage, future)

```
ChainWorld → mjSpec:
  1. mj_makeSpec()
  2. Add floor, lights, materials
  3. For each Chain (primary first):
     - First bead: mjs_addBody(worldbody) + freejoint
     - Each subsequent bead: mjs_addBody(parent_bead) + ball joint
     - Ball joints provide 3 rotational DOF (simulates string pivot)
     - Joint angle limits prevent block interpenetration
  4. For junction blocks (multi-chain branch points):
     - Secondary chains: own nested hierarchy + mjEQ_CONNECT to junction body
  5. Configure collision filtering (parent-child auto + exclude pairs + callback)
  6. mj_compile() → mjModel
```

---

## 11. Differences from progui V1

| Aspect | progui (V1) | ChainMaker (V2) |
|--------|------------|-----------------|
| Chains | Single chain only | Multiple chains with junctions |
| Body hierarchy | Nested (child of parent) | Nested per chain + `mjEQ_CONNECT` at junctions |
| String model | Ball joints (nested hierarchy) | Ball joints (nested hierarchy) — same proven approach |
| Physics during build | mj_recompile per block | No physics at all |
| Rendering during build | Full MuJoCo model render | Visualization-only (mjv_initGeom) |
| Performance | O(n²) — recompile grows with model | O(1) per block placement |
| Data model | Linear list of ChainEntry | 3D grid map + chain paths |
| Turn blocks | Implicit (direction change) | Explicit (marked in grid, exclusive) |
| Save format | Direction tokens | JSON (machine-readable for AI/program export) |

---

## 12. Implementation Roadmap

| Phase | Focus | Key Deliverable |
|-------|-------|----------------|
| **Phase 1** | Build Stage GUI | Multi-chain placement, grid rendering, save/load JSON |
| **Phase 2** | Simulation Stage | Grid→mjSpec compiler, physics with profiling overlay |
| **Phase 3** | Linear Actuators | Solenoid/motor modeling, sensor suite, PID control |
| **Phase 4** | Sensor Dashboard | Real-time force/energy/current plots, CSV export |
| **Phase 5** | Multi-Robot | mjs_attach composition, parallel simulation, coordinated control |
| **Phase 6** | Automation | Scripted trajectories, movement optimization, batch sweeps |
| **Phase 7** | Manufacturing Export | Bill of materials, assembly instructions, 3D print files |

---

## 13. Open Questions / Future Work

### Build Stage
- **Variable bead sizes per chain** — future iteration
- **Non-90° turns** (30°, 60°, 180°) — future iteration
- **Non-cube bead shapes** (triangular wedge for turns) — future iteration
- **Undo/redo stack** — desirable but not V1 requirement
- **3D grid visualization** (subtle grid lines in viewport) — nice to have
- **Chain coloring strategy** — auto-assign from a palette, or user-selectable?
- **Maximum grid dimensions** — should there be bounds, or infinite?
- **Import from progui format** — not supported; progui is reference code only

### Simulation Stage
- **Optimal `MIN_COLLISION_DISTANCE`** — needs empirical tuning (start with 3)
- ~~**Tendon model fidelity**~~ — **RESOLVED:** Structural string uses nested body hierarchy 
  with ball joints, NOT tendons. Tendons are Phase 3 (actuator cable transmission) only.
- ~~**Joint type selection**~~ — **RESOLVED:** Ball joints (`mjJNT_BALL`) for chain connections. 
  Provides 3 rotational DOF (string-like pivot), zero translational DOF (cannot stretch).
- **Contact `condim`** — frictionless (1) vs. frictional (3) for block contacts
- **Solver iteration count** — balance between accuracy and speed at 1000+ blocks
- **Island decomposition effectiveness** — how many independent islands do chain structures produce?

### Actuators & Sensors
- **Actuator placement UX** — how does the user specify where actuators go?
- **Electrical model calibration** — where do motor constants (K_force, R, K_emf) come from?
- **Sensor data visualization** — real-time plots, dashboard layout, export format
- **Force feedback** — can we estimate the "feel" of the structure under load?
- **Thermal modeling** — should we track motor heating over long simulation runs?

### Multi-Robot
- **Maximum feasible robot count** — empirical testing needed (100 may require GPU/MJX)
- **Inter-robot communication model** — how do robots coordinate in simulation?
- **Collision groups for robot boundaries** — prevent internal block collisions between robots
- **Scene management** — spawning, removing, duplicating robots at runtime
- **Performance LOD threshold** — at what distance to switch rendering fidelity?

### Manufacturing
- **Bead manufacturing tolerance** — how does simulation parameter accuracy map to real parts?
- **String tension specification** — what material, what pre-tension?
- **Assembly tooling** — can we generate assembly instructions from chain data?
- **Standard parts catalog** — define the set of mass-manufactured components
