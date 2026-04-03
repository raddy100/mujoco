"""
bench_sim.py  —  ChainMaker simulation performance benchmark

Builds the same 40-block chain model that CompileWorld() generates and tests
many parameter combinations.  Prints a ranked table showing wall-clock time
per step and RT ratio.

Run:  python bench_sim.py
"""

import math
import time
import json
import os
import sys
import textwrap

import mujoco
import numpy as np

# ---------------------------------------------------------------------------
# Parameters matching chainmaker_compile.h / chainmaker.h defaults
# ---------------------------------------------------------------------------

N_BLOCKS     = 40        # number of blocks (use 40 for benchmark per user request)
BEAD_SIZE    = 0.05      # metres (default kDefaultBeadSize)
GAP_RATIO    = 0.05      # (default kDefaultGapRatio)
JOINT_DAMP   = 0.15      # kJointDamping
GEOM_MARGIN  = 0.0       # kGeomMargin
KMIN_DIST    = 3         # kMinCollisionDist (neighbour filter distance)
WARMUP_STEPS = 100       # steps to warm up before measuring
MEASURE_STEPS= 500       # steps to measure over

HALF  = BEAD_SIZE / 2.0
GAP   = BEAD_SIZE * GAP_RATIO
STEP  = BEAD_SIZE * (1.0 + GAP_RATIO)   # cell stride


def cone_limit_deg(half_size: float, gap: float) -> float:
    """Mirrors ComputeConeLimit() in chainmaker_compile.cpp."""
    a   = half_size
    c   = 1.0 + (gap / a if a > 0 else 0.0)
    sq2 = math.sqrt(2.0)
    if c >= sq2:
        phi = math.pi / 2.0
    else:
        s   = max(0.0, min(1.0, c / sq2))
        phi = math.asin(s) - math.pi / 4.0
    phi = max(math.radians(5.0), min(math.radians(80.0), phi))
    return math.degrees(phi) * 0.5  # halved, same as compile code


def build_chain_xml(n_blocks: int) -> str:
    """
    Returns MJCF XML for an n-block straight chain along +X.

    Mirrors CompileWorld:
     - worldbody  → floor plane
     - root body   → free joint + box geom
     - body[1..N-1] → ball joint + box geom (parent = previous body)
     - ball joints have cone limits
     - condim=1 (frictionless block-block, structural mode)
     - exclude grandparent↔grandchild pairs (same as compile code)
    """
    cone_deg = cone_limit_deg(HALF, GAP)
    half     = HALF
    step     = STEP
    damp     = JOINT_DAMP

    # Contact parameters (kSolref / kSolimp)
    solref = "0.15 0.7"
    solimp = "0.9 0.95 0.001 0.5 2.0"

    lines = []
    lines.append('<mujoco model="chain_bench">')
    lines.append('  <option/>')
    lines.append('  <worldbody>')
    # Floor
    lines.append(
        f'    <geom type="plane" size="25 25 0.1" contype="1" conaffinity="7" '
        f'rgba="0.5 0.5 0.5 1" solref="{solref}" solimp="{solimp}"/>'
    )

    indent = "    "
    for i in range(n_blocks):
        if i == 0:
            # Root: free joint, position lifted above floor
            px = 0.0
            py = 0.0
            pz = BEAD_SIZE  # lifted by one bead
            lines.append(
                f'{indent}<body name="b{i}" pos="{px} {py} {pz}">'
            )
            lines.append(
                f'{indent}  <joint type="free" damping="{damp}"/>'
            )
        else:
            # Child: offset along +X by one step, ball joint
            indent = "  " * (i + 2)
            ax     = -1 * (half + GAP * 0.5)  # anchor on -X face
            lines.append(
                f'{indent}<body name="b{i}" pos="{step} 0 0">'
            )
            lines.append(
                f'{indent}  <joint type="ball" damping="{damp}" '
                f'pos="{ax} 0 0" limited="true" range="0 {cone_deg:.4f}"/>'
            )

        lines.append(
            f'{indent}  <geom type="box" size="{half} {half} {half}" '
            f'condim="1" contype="1" conaffinity="1" '
            f'solref="{solref}" solimp="{solimp}" rgba="0.2 0.6 1.0 1.0"/>'
        )

    # Close all body tags
    for i in range(n_blocks - 1, -1, -1):
        indent = "  " * (i + 2)
        lines.append(f'{indent}</body>')

    # Grandparent-grandchild exclusions
    lines.append('  </worldbody>')
    lines.append('  <contact>')
    for i in range(n_blocks - 2):
        lines.append(f'    <exclude body1="b{i}" body2="b{i+2}"/>')
    lines.append('  </contact>')
    lines.append('</mujoco>')

    return "\n".join(lines)


def run_benchmark(xml: str, label: str, **opt_overrides) -> dict:
    """
    Loads the model, applies option overrides, runs WARMUP_STEPS steps,
    then times MEASURE_STEPS steps.  Returns a dict of metrics.
    """
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)

    # Apply option overrides
    for key, val in opt_overrides.items():
        setattr(m.opt, key, val)

    # Warmup
    for _ in range(WARMUP_STEPS):
        mujoco.mj_step(m, d)

    # Reset timers
    n_timers = int(mujoco.mjtTimer.mjNTIMER)
    for i in range(n_timers):
        d.timer[i].duration = 0.0
        d.timer[i].number   = 0

    # Measure
    t0 = time.perf_counter()
    for _ in range(MEASURE_STEPS):
        mujoco.mj_step(m, d)
    wall_s = time.perf_counter() - t0

    wall_ms_per_step = wall_s / MEASURE_STEPS * 1000.0
    sim_per_step_ms  = m.opt.timestep * 1000.0
    rt_ratio         = sim_per_step_ms / wall_ms_per_step

    def timer_ms(slot):
        n = d.timer[slot].number
        return (d.timer[slot].duration / n) if n > 0 else 0.0

    return {
        "label"       : label,
        "wall_ms"     : wall_ms_per_step,
        "rt_ratio"    : rt_ratio,
        "ncon"        : int(d.ncon),
        "collision_ms": timer_ms(mujoco.mjtTimer.mjTIMER_POS_COLLISION),
        "solver_ms"   : timer_ms(mujoco.mjtTimer.mjTIMER_CONSTRAINT),
        "kin_ms"      : timer_ms(mujoco.mjtTimer.mjTIMER_POS_KINEMATICS),
        "inertia_ms"  : timer_ms(mujoco.mjtTimer.mjTIMER_POS_INERTIA),
        "timestep"    : m.opt.timestep,
        "iterations"  : m.opt.iterations,
        "solver"      : int(m.opt.solver),
        "integrator"  : int(m.opt.integrator),
        "nbody"       : m.nbody,
        "nv"          : m.nv,
        "disableflags": int(m.opt.disableflags),
    }


# ---------------------------------------------------------------------------
# Benchmark configurations to test
# ---------------------------------------------------------------------------

def make_configs():
    """Returns list of (label, override_dict) pairs to benchmark."""
    newton = int(mujoco.mjtSolver.mjSOL_NEWTON)
    cg     = int(mujoco.mjtSolver.mjSOL_CG)
    pgs    = int(mujoco.mjtSolver.mjSOL_PGS)
    ifast  = int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
    euler  = int(mujoco.mjtIntegrator.mjINT_EULER)
    sparse = int(mujoco.mjtJacobian.mjJAC_SPARSE)
    dense  = int(mujoco.mjtJacobian.mjJAC_DENSE)
    no_con = int(mujoco.mjtDisableBit.mjDSBL_CONTACT)

    configs = [
        # ---- Baseline (current defaults in chainmaker_compile.cpp) ----------
        ("01_baseline",
         dict(timestep=0.002, iterations=20, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        # ---- Larger timestep only ------------------------------------------
        ("02_dt005_itr20",
         dict(timestep=0.005, iterations=20, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        ("03_dt010_itr20",
         dict(timestep=0.010, iterations=20, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        ("04_dt020_itr20",
         dict(timestep=0.020, iterations=20, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        # ---- Fewer iterations only -----------------------------------------
        ("05_dt002_itr10",
         dict(timestep=0.002, iterations=10, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        ("06_dt002_itr05",
         dict(timestep=0.002, iterations=5,  tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        # ---- Combined timestep + fewer iters (sweet spots) -----------------
        ("07_dt005_itr10",
         dict(timestep=0.005, iterations=10, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        ("08_dt005_itr05",
         dict(timestep=0.005, iterations=5,  tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        ("09_dt010_itr05",
         dict(timestep=0.010, iterations=5,  tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        ("10_dt010_itr10",
         dict(timestep=0.010, iterations=10, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse)),

        # ---- Looser tolerance -----------------------------------------------
        ("11_dt005_itr10_tol1e4",
         dict(timestep=0.005, iterations=10, tolerance=1e-4,
              solver=newton, integrator=ifast, jacobian=sparse)),

        ("12_dt010_itr10_tol1e4",
         dict(timestep=0.010, iterations=10, tolerance=1e-4,
              solver=newton, integrator=ifast, jacobian=sparse)),

        # ---- CG solver (cheaper per iteration) -----------------------------
        ("13_cg_dt005_itr20",
         dict(timestep=0.005, iterations=20, tolerance=1e-6,
              solver=cg, integrator=ifast, jacobian=sparse)),

        ("14_cg_dt010_itr10",
         dict(timestep=0.010, iterations=10, tolerance=1e-6,
              solver=cg, integrator=ifast, jacobian=sparse)),

        # ---- Disable contacts (pure skeletal, no floor bounce) -------------
        ("15_nocontact_dt005_itr10",
         dict(timestep=0.005, iterations=10, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse,
              disableflags=no_con)),

        ("16_nocontact_dt010_itr05",
         dict(timestep=0.010, iterations=5,  tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse,
              disableflags=no_con)),

        ("17_nocontact_dt020_itr05",
         dict(timestep=0.020, iterations=5,  tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=sparse,
              disableflags=no_con)),

        # ---- Euler integrator (less stable but cheaper for low-fidelity) ---
        ("18_euler_dt005_itr10",
         dict(timestep=0.005, iterations=10, tolerance=1e-6,
              solver=newton, integrator=euler, jacobian=sparse)),

        # ---- Dense Jacobian (faster for small nv) --------------------------
        ("19_dense_dt005_itr10",
         dict(timestep=0.005, iterations=10, tolerance=1e-6,
              solver=newton, integrator=ifast, jacobian=dense)),

        # ---- Best-guess sweet spot based on analysis -----------------------
        ("20_PROPOSED_dt008_itr08_tol1e5",
         dict(timestep=0.008, iterations=8,  tolerance=1e-5,
              solver=newton, integrator=ifast, jacobian=sparse)),
    ]
    return configs


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    xml = build_chain_xml(N_BLOCKS)

    # Save the test chain as JSON (for loading into the real ChainMaker app)
    chain_json = {
        "version": 1,
        "bead_size": BEAD_SIZE,
        "gap_ratio": GAP_RATIO,
        "chains": [{
            "id": 0,
            "name": "Chain A",
            "color": [0.2, 0.6, 1.0, 1.0],
            "head_direction": 0,
            "blocks": [
                {"pos": [i, 0, 0], "is_turn": False,
                 "turn_entry_axis": -1, "turn_exit_axis": -1}
                for i in range(N_BLOCKS)
            ]
        }]
    }
    json_path = os.path.join(os.path.dirname(__file__), "bench_40blocks.json")
    with open(json_path, "w") as f:
        json.dump(chain_json, f, indent=2)
    print(f"[bench] Wrote {N_BLOCKS}-block chain to {json_path}")

    print(f"\n[bench] Building MuJoCo model: {N_BLOCKS} blocks, "
          f"nbody=~{N_BLOCKS+1}, nv=~{6+39*3}")
    print(f"[bench] Warmup={WARMUP_STEPS} steps, Measure={MEASURE_STEPS} steps\n")

    configs = make_configs()
    results = []

    for label, overrides in configs:
        sys.stdout.write(f"  Running {label} ... ")
        sys.stdout.flush()
        try:
            r = run_benchmark(xml, label, **overrides)
            results.append(r)
            sys.stdout.write(
                f"{r['wall_ms']:.3f} ms/step  RT={r['rt_ratio']:.2f}x  "
                f"ncon={r['ncon']}\n"
            )
        except Exception as e:
            sys.stdout.write(f"FAILED: {e}\n")

    # Sort by wall_ms (fastest first)
    results.sort(key=lambda r: r["wall_ms"])

    baseline = next((r for r in results if r["label"] == "01_baseline"), None)

    print("\n" + "=" * 90)
    print(f"{'Rank':<5} {'Config':<38} {'ms/step':>8} {'RT':>7} "
          f"{'Speedup':>8} {'ncon':>5} {'dt':>6} {'itr':>4} {'Col%':>5} {'Sol%':>5}")
    print("=" * 90)

    for rank, r in enumerate(results, 1):
        speedup = (baseline["wall_ms"] / r["wall_ms"]) if baseline else 1.0
        total   = r["wall_ms"]
        col_pct = 100 * r["collision_ms"] / total if total > 0 else 0
        sol_pct = 100 * r["solver_ms"]    / total if total > 0 else 0
        flag    = " ★" if rank == 1 else ("  [baseline]" if r["label"] == "01_baseline" else "")
        print(
            f"{rank:<5} {r['label']:<38} {r['wall_ms']:>8.3f} {r['rt_ratio']:>7.2f}x"
            f" {speedup:>7.1f}x {r['ncon']:>5} {r['timestep']:>6.4f} "
            f"{r['iterations']:>4}  {col_pct:>4.0f}%  {sol_pct:>4.0f}%{flag}"
        )

    print("=" * 90)

    best = results[0]
    print(f"\n[bench] WINNER: {best['label']}")
    print(f"  wall_ms    = {best['wall_ms']:.3f} ms/step")
    print(f"  RT ratio   = {best['rt_ratio']:.2f}x real-time")
    print(f"  timestep   = {best['timestep']}")
    print(f"  iterations = {best['iterations']}")
    print(f"  ncon       = {best['ncon']}")
    if baseline:
        print(f"  Speedup    = {baseline['wall_ms'] / best['wall_ms']:.1f}x vs baseline")

    print("\n[bench] Key observations:")
    if baseline:
        col_baseline = baseline["collision_ms"] / baseline["wall_ms"] * 100 if baseline["wall_ms"] else 0
        sol_baseline = baseline["solver_ms"]    / baseline["wall_ms"] * 100 if baseline["wall_ms"] else 0
        print(f"  Baseline bottleneck: collision={col_baseline:.0f}%  solver={sol_baseline:.0f}%")
        print(f"  Baseline ncon={baseline['ncon']}, nv={baseline['nv']}, nbody={baseline['nbody']}")

    # Compute steps-per-60hz-frame for best vs baseline
    frames_60hz = 1.0 / 60.0 * 1000.0  # 16.67ms budget
    if baseline:
        steps_base = frames_60hz / baseline["wall_ms"]
        steps_best = frames_60hz / best["wall_ms"]
        sim_adv_base = steps_base * baseline["timestep"] * 1000.0
        sim_adv_best = steps_best * best["timestep"] * 1000.0
        print(f"\n  At 60 Hz display budget (16.67ms/frame):")
        print(f"  Baseline: ~{steps_base:.1f} steps/frame = {sim_adv_base:.1f}ms sim advance")
        print(f"  Best:     ~{steps_best:.1f} steps/frame = {sim_adv_best:.1f}ms sim advance")


if __name__ == "__main__":
    main()
