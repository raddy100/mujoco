"""
ChainMaker IPC Test Suite
=========================
Exercises the ChainMaker GUI entirely through the IPC socket — no pyautogui,
no keyboard injection, no window focus issues.

Usage:
    python test_chainmaker.py

ChainMaker must NOT be running already (the script launches it with --test).
"""

import os
import sys
import time
import subprocess

from test_chainmaker_client import launch_chainmaker, DEFAULT_EXE

SCREENSHOT_DIR = os.path.join(os.path.dirname(__file__), "test_screenshots")
os.makedirs(SCREENSHOT_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Minimal test framework
# ---------------------------------------------------------------------------

passed = []
failed = []

def ok(name: str, condition: bool, detail: str = ""):
    if condition:
        passed.append(name)
        print(f"  [PASS] {name}" + (f" — {detail}" if detail else ""))
    else:
        failed.append(name)
        print(f"  [FAIL] {name}" + (f" — {detail}" if detail else ""))
    return condition

def section(title: str):
    print(f"\n{'='*60}\n  {title}\n{'='*60}")

# ---------------------------------------------------------------------------
# Screenshot helper
# ---------------------------------------------------------------------------

def ss(c, name: str) -> dict:
    path = os.path.join(SCREENSHOT_DIR, f"{name}.ppm")
    r = c.screenshot(path)
    if r.get("ok"):
        print(f"    [screenshot] {path}")
    return r

# ---------------------------------------------------------------------------
# Main test runner
# ---------------------------------------------------------------------------

print("\n=== ChainMaker IPC Test Suite ===\n")

# Make sure no old instance is running
for proc in ['ChainMaker', 'ChainMaker.exe']:
    try:
        subprocess.run(['taskkill', '/F', '/IM', proc],
                       capture_output=True, check=False)
    except Exception:
        pass
time.sleep(0.5)

with launch_chainmaker(exe_path=DEFAULT_EXE, wait=3.0) as c:

    # ------------------------------------------------------------------
    section("1. Connection and initial state")
    # ------------------------------------------------------------------

    state = c.get_state()
    ok("get_state returns ok", state.get("ok") is True, str(state))
    ok("initial mode is BUILD", state.get("mode") == "BUILD")
    ok("initial nchains == 1", state.get("nchains") == 1)
    ok("initial nblocks == 1", state.get("nblocks") == 1,
       f"nblocks={state.get('nblocks')}")
    ok("initial active_chain == 0", state.get("active_chain") == 0)
    ok("bead_size is a number", isinstance(state.get("bead_size"), (int, float)))

    ss(c, "01_initial_state")

    # ------------------------------------------------------------------
    section("2. Place blocks (build mode)")
    # ------------------------------------------------------------------

    r = c.place_block()
    ok("place_block 1 succeeds", r.get("ok") is True, str(r))
    ok("nblocks is 2", r.get("nblocks") == 2)

    r = c.place_block()
    ok("place_block 2 succeeds", r.get("ok") is True)
    ok("nblocks is 3", r.get("nblocks") == 3)

    r = c.place_block()
    ok("place_block 3 succeeds", r.get("ok") is True)

    state = c.get_state()
    ok("state shows 4 blocks after 3 places", state.get("nblocks") == 4)

    ss(c, "02_four_blocks")

    # ------------------------------------------------------------------
    section("3. Direction changes")
    # ------------------------------------------------------------------

    r = c.set_direction("POS_Y")
    ok("set_direction POS_Y", r.get("ok") is True)

    r = c.place_block()
    ok("place block in +Y direction", r.get("ok") is True)

    r = c.set_direction("NEG_X")
    ok("set_direction NEG_X", r.get("ok") is True)

    r = c.place_block()
    ok("place block in -X direction", r.get("ok") is True)

    r = c.set_direction("POS_Z")
    ok("set_direction POS_Z", r.get("ok") is True)

    r = c.place_block()
    ok("place block in +Z direction", r.get("ok") is True)

    r = c.set_direction("INVALID_DIR")
    ok("set_direction with bad dir returns error", r.get("ok") is False,
       str(r))

    state = c.get_state()
    ok("7 blocks total after direction moves", state.get("nblocks") == 7,
       f"nblocks={state.get('nblocks')}")

    ss(c, "03_direction_changes")

    # ------------------------------------------------------------------
    section("4. Delete block")
    # ------------------------------------------------------------------

    r = c.delete_block()
    ok("delete_block succeeds", r.get("ok") is True)
    ok("nblocks decreased to 6", r.get("nblocks") == 6,
       f"nblocks={r.get('nblocks')}")

    ss(c, "04_after_delete")

    # ------------------------------------------------------------------
    section("5. Gap and bead size")
    # ------------------------------------------------------------------

    state = c.get_state()
    orig_gap = state.get("gap_ratio", 0.05)

    r = c.adjust_gap(0.05)
    ok("adjust_gap +0.05 ok", r.get("ok") is True)
    ok("gap_ratio increased", r.get("gap_ratio", 0) > orig_gap,
       f"new={r.get('gap_ratio')}")

    r = c.adjust_gap(-0.05)
    ok("adjust_gap -0.05 ok", r.get("ok") is True)

    r = c.set_bead_size(0.08)
    ok("set_bead_size 0.08 ok", r.get("ok") is True)

    r = c.set_bead_size(0.05)
    ok("set_bead_size back to 0.05 ok", r.get("ok") is True)

    r = c.set_bead_size(99.0)
    ok("set_bead_size out-of-range returns error", r.get("ok") is False)

    # ------------------------------------------------------------------
    section("6. Save and load")
    # ------------------------------------------------------------------

    save_path = os.path.join(os.path.dirname(DEFAULT_EXE), "test_ipc_save.json")

    r = c.save(save_path)
    ok("save returns ok", r.get("ok") is True, str(r))
    ok("save file exists on disk", os.path.exists(save_path),
       f"path={save_path}")

    # Reset and reload
    r = c.reset()
    ok("reset ok", r.get("ok") is True)
    state = c.get_state()
    ok("after reset nblocks == 1", state.get("nblocks") == 1)

    r = c.load(save_path)
    ok("load returns ok", r.get("ok") is True, str(r))

    state = c.get_state()
    ok("after load nblocks == 6", state.get("nblocks") == 6,
       f"nblocks={state.get('nblocks')}")

    ss(c, "05_after_load")

    # ------------------------------------------------------------------
    section("7. Multi-chain workflow")
    # ------------------------------------------------------------------

    # Ensure we have at least one block at (0,0,0)
    c.reset()
    # Place a few blocks so we have a junction candidate
    for _ in range(3):
        c.place_block()

    r = c.start_chain_at(0, 0, 0)
    ok("start_chain_at origin succeeds", r.get("ok") is True, str(r))

    state = c.get_state()
    ok("now have 2 chains", state.get("nchains") == 2,
       f"nchains={state.get('nchains')}")
    ok("active chain is 1", state.get("active_chain") == 1,
       f"active={state.get('active_chain')}")

    # Place blocks on the new chain in a different direction
    r = c.set_direction("POS_Y")
    ok("set direction on chain 2", r.get("ok") is True)
    r = c.place_block()
    ok("place block on chain 2", r.get("ok") is True)

    r = c.switch_chain(0)
    ok("switch back to chain 0", r.get("ok") is True)
    state = c.get_state()
    ok("active chain is now 0", state.get("active_chain") == 0)

    r = c.switch_chain(999)
    ok("switch to invalid chain returns error", r.get("ok") is False)

    ss(c, "06_two_chains")

    # ------------------------------------------------------------------
    section("8. New-chain-pick mode and cancel")
    # ------------------------------------------------------------------

    r = c.new_chain_mode()
    ok("new_chain_mode enters pick mode", r.get("ok") is True)
    state = c.get_state()
    ok("mode is NEW_CHAIN_PICK", state.get("mode") == "NEW_CHAIN_PICK",
       f"mode={state.get('mode')}")

    r = c.cancel_pick()
    ok("cancel_pick returns ok", r.get("ok") is True)
    state = c.get_state()
    ok("mode returns to BUILD after cancel", state.get("mode") == "BUILD")

    # ------------------------------------------------------------------
    section("9. Simulation mode")
    # ------------------------------------------------------------------

    # Reset to a clean 5-block world for compilation
    c.reset()
    for _ in range(4):
        c.place_block()

    r = c.enter_simulate()
    ok("enter_simulate returns ok", r.get("ok") is True, str(r))
    state = c.get_state()
    ok("mode is SIMULATE", state.get("mode") == "SIMULATE",
       f"mode={state.get('mode')}")
    ok("nbody > 1", state.get("nbody", 0) > 1,
       f"nbody={state.get('nbody')}")

    time.sleep(0.5)  # let physics run a moment
    ss(c, "07_simulation_mode")

    r = c.exit_simulate()
    ok("exit_simulate returns ok", r.get("ok") is True)
    state = c.get_state()
    ok("mode returns to BUILD", state.get("mode") == "BUILD")

    ss(c, "08_back_to_build")

    # ------------------------------------------------------------------
    section("9a. Multi-chain simulation (regression for mjEQ_CONNECT crash)")
    # ------------------------------------------------------------------
    # Build chain 0 along +X, then chain 1 branching off the origin in +Y
    c.reset()
    for _ in range(3):
        c.place_block()                  # chain 0: (0,0,0) → (1,0,0) → (2,0,0) → (3,0,0)

    r = c.start_chain_at(0, 0, 0)       # chain 1 junction at origin
    ok("9a: start_chain_at succeeds", r.get("ok") is True, str(r))
    c.set_direction("POS_Y")
    c.place_block()                      # chain 1: (0,0,0) → (0,1,0)
    c.place_block()                      # chain 1: → (0,2,0)

    state = c.get_state()
    ok("9a: 2 chains built", state.get("nchains") == 2, f"nchains={state.get('nchains')}")

    r = c.enter_simulate()
    ok("9a: multi-chain enter_simulate ok (no compile error)", r.get("ok") is True, str(r))

    state = c.get_state()
    ok("9a: mode is SIMULATE", state.get("mode") == "SIMULATE", f"mode={state.get('mode')}")
    ok("9a: nbody > 5", state.get("nbody", 0) > 5, f"nbody={state.get('nbody')}")

    time.sleep(0.5)
    ss(c, "09a_multi_chain_sim")

    r = c.exit_simulate()
    ok("9a: exit_simulate ok", r.get("ok") is True)

    # ------------------------------------------------------------------
    section("9d. Profiler - default preset is Accurate (Newton, iterations=50)")
    # ------------------------------------------------------------------
    # Build a 6-block chain and verify profiler reports Accurate preset defaults
    c.reset()
    for _ in range(5):
        c.place_block()

    c.enter_simulate()
    time.sleep(0.3)  # let a few steps accumulate timer data

    r = c.get_profiler()
    ok("9d: get_profiler ok in sim mode", r.get("ok") is True, str(r))
    ok("9d: default preset = Accurate (iterations=50)", r.get("iterations") == 50,
       f"iterations={r.get('iterations')}")
    ok("9d: sim_preset_label = Accurate", r.get("sim_preset_label") == "Accurate",
       f"sim_preset_label={r.get('sim_preset_label')}")
    ok("9d: step_ms reported (timer working)",
       isinstance(r.get("step_ms"), (int, float)),
       f"step_ms={r.get('step_ms')}")
    ok("9d: nv > 0", r.get("nv", 0) > 0, f"nv={r.get('nv')}")

    r2 = c.get_profiler()   # not in sim → error expected after exit
    c.exit_simulate()
    r3 = c.get_profiler()
    ok("9d: get_profiler outside sim returns error",
       r3.get("ok") is False, str(r3))

    ss(c, "09d_profiler")

    # ------------------------------------------------------------------
    section("9d2. Sim preset switching -- Accurate -> Balanced -> Fast -> Accurate")
    # ------------------------------------------------------------------
    c.reset()
    for _ in range(5):
        c.place_block()

    # Set preset to Balanced before entering sim
    c.set_sim_preset(1)  # 1 = Balanced
    c.enter_simulate()
    time.sleep(0.3)

    r = c.get_profiler()
    ok("9d2: Balanced preset iterations=25", r.get("iterations") == 25,
       f"iterations={r.get('iterations')}")
    ok("9d2: Balanced preset label", r.get("sim_preset_label") == "Balanced",
       f"label={r.get('sim_preset_label')}")

    # Switch to Fast live (mid-simulation)
    c.set_sim_preset(2)  # 2 = Fast
    time.sleep(0.1)
    r = c.get_profiler()
    ok("9d2: Fast preset iterations=20 (live switch)", r.get("iterations") == 20,
       f"iterations={r.get('iterations')}")
    ok("9d2: Fast preset label", r.get("sim_preset_label") == "Fast",
       f"label={r.get('sim_preset_label')}")

    # Switch back to Accurate live
    c.set_sim_preset(0)  # 0 = Accurate
    time.sleep(0.1)
    r = c.get_profiler()
    ok("9d2: Accurate preset iterations=50 (live switch back)", r.get("iterations") == 50,
       f"iterations={r.get('iterations')}")
    ok("9d2: Accurate preset label", r.get("sim_preset_label") == "Accurate",
       f"label={r.get('sim_preset_label')}")

    c.exit_simulate()
    ss(c, "09d2_preset_switch")

    ss(c, "09d_profiler")

    # ------------------------------------------------------------------
    section("9e. Recording status IPC (no-ffmpeg graceful path)")
    # ------------------------------------------------------------------
    # Test the IPC command plumbing. We don't require ffmpeg to be installed;
    # start_recording may fail, but the IPC protocol must work correctly.
    c.reset()
    for _ in range(3):
        c.place_block()

    # Not recording initially
    r = c.get_recording_status()
    ok("9e: get_recording_status ok", r.get("ok") is True, str(r))
    ok("9e: not recording initially", r.get("is_recording") is False,
       str(r))

    # Try start; may fail if ffmpeg is absent or the first frame write fails
    r_start = c.start_recording("chainmaker_test_output.mp4")
    ok("9e: start_recording returns ok field (not crash)",
       "ok" in r_start, str(r_start))
    ok("9e: start_recording returns path field (not crash)",
       "path" in r_start or "error" in r_start, str(r_start))

    # stop_recording is always safe (even if recording already stopped internally)
    r_stop = c.stop_recording()
    ok("9e: stop_recording returns ok", r_stop.get("ok") is True, str(r_stop))

    # Double-stop is a no-op (not an error)
    r_stop2 = c.stop_recording()
    ok("9e: double stop_recording returns ok", r_stop2.get("ok") is True, str(r_stop2))

    # After stop, not recording
    r_final = c.get_recording_status()
    ok("9e: not recording after stop", r_final.get("is_recording") is False,
       str(r_final))

    ss(c, "09e_recording_status")

    # ------------------------------------------------------------------
    section("9f. Weld self-intersection: loop chain compiles with mjEQ_WELD")
    # ------------------------------------------------------------------
    # Build a chain that loops back onto itself — verified by entering simulate
    # without a compile error. The weld constraint should close the loop.
    # Chain: origin→+X×2→+Y×2→-X×2→-Y×1 which arrives at (0,1,0);
    # then -Y places at (0,0,0) which is occupied — triggers WELD.
    c.reset()
    c.set_direction("+X")
    c.place_block()   # (1,0,0)
    c.place_block()   # (2,0,0)
    c.set_direction("+Y")
    c.place_block()   # (2,1,0)
    c.place_block()   # (2,2,0)
    c.set_direction("-X")
    c.place_block()   # (1,2,0)
    c.place_block()   # (0,2,0)
    c.set_direction("-Y")
    c.place_block()   # (0,1,0)
    c.place_block()   # (0,0,0) → self-intersection, WELD added

    r = c.enter_simulate()
    ok("9f: looped chain enters simulate (weld compiles ok)",
       r.get("ok") is True, str(r))
    ok("9f: nbody >= 10 (9 unique blocks + world)",
       r.get("nbody", 0) >= 10, f"nbody={r.get('nbody')}")

    state = c.get_state()
    ok("9f: mode is SIMULATE", state.get("mode") == "SIMULATE",
       f"mode={state.get('mode')}")

    time.sleep(0.3)
    ss(c, "09f_weld_self_intersect")
    c.exit_simulate()

    # ------------------------------------------------------------------
    section("9g. Cross-chain PlaceBlock junction: no duplicate bodies")
    # ------------------------------------------------------------------
    # Bug regression: when chain 1 passes THROUGH a block already owned by
    # chain 0, the old code created a duplicate body at the junction (visual
    # overlap).  The fix uses mjEQ_CONNECT (ball-and-socket) + skip instead.
    #
    # Layout (grid coords):
    #  Chain 0: (0,0,0)→+X→(1,0,0)→(2,0,0)→(3,0,0)
    #  Chain 1: junction at (1,0,0), extends +Y:
    #    (1,0,0)→(1,1,0)→(1,2,0)→turn-X→(0,2,0)→turn-Y→(0,1,0)
    #    → PlaceBlock hits (0,0,0) [chain0 block] → cross-chain junction
    #    → places new block at (0,-1,0) [beyond junction]
    #
    # Expected nbody = 10:
    #   worldbody(1) + chain0 root+3 bodies(4) + chain1 5 bodies(5, no dup) = 10
    c.reset()
    c.set_direction("+X")
    for _ in range(3):
        c.place_block()  # chain 0: (0,0,0)→(1,0,0)→(2,0,0)→(3,0,0)

    r = c.start_chain_at(1, 0, 0)  # chain 1 starts at junction (1,0,0)
    ok("9g: start_chain_at(1,0,0) ok", r.get("ok") is True, str(r))

    c.place_block()          # (1,1,0)
    c.place_block()          # (1,2,0)
    c.set_direction("-X")
    c.place_block()          # (0,2,0)   — turn block at (1,2,0)
    c.set_direction("-Y")
    c.place_block()          # (0,1,0)   — turn block at (0,2,0)
    r = c.place_block()      # crosses through (0,0,0) [chain 0], places (0,-1,0)
    ok("9g: cross-chain PlaceBlock succeeds", r.get("ok") is True, str(r))

    state = c.get_state()
    ok("9g: 2 chains", state.get("nchains") == 2, f"nchains={state.get('nchains')}")

    r = c.enter_simulate()
    ok("9g: enter_simulate (cross-chain CONNECT constraint) ok",
       r.get("ok") is True, str(r))
    # With the fix: 1 worldbody + 4 chain0 bodies + 5 chain1 bodies (no dup at junction)
    ok("9g: nbody=10 (no duplicate junction body)",
       r.get("nbody") == 10, f"nbody={r.get('nbody')}")

    state = c.get_state()
    ok("9g: mode is SIMULATE", state.get("mode") == "SIMULATE",
       f"mode={state.get('mode')}")

    time.sleep(0.3)
    ss(c, "09g_cross_chain_connect")
    c.exit_simulate()

    # ------------------------------------------------------------------
    section("9b. Self-intersection ghost position (regression)")
    # ------------------------------------------------------------------
    # Build a small L-shaped chain that approaches its own origin:
    #   (0,0,0) → (1,0,0) → (2,0,0)   direction +X
    #   turn +Y → (2,1,0) → (2,2,0)
    #   turn -X → (1,2,0) → (0,2,0)
    #   turn -Y → (0,1,0) → head at (0,1,0), next = (0,0,0) which is OCCUPIED
    #   Ghost should be at (0,-1,0), NOT (0,0,0)
    c.reset()
    c.set_direction("+X")
    c.place_block()   # (1,0,0)
    c.place_block()   # (2,0,0)
    c.set_direction("+Y")
    c.place_block()   # (2,1,0)
    c.place_block()   # (2,2,0)
    c.set_direction("-X")
    c.place_block()   # (1,2,0)
    c.place_block()   # (0,2,0)
    c.set_direction("-Y")
    c.place_block()   # (0,1,0)
    # Head is now at (0,1,0), direction -Y, next = (0,0,0) which IS occupied
    state = c.get_state()
    head  = state.get("head", {})
    ghost = state.get("ghost", {})
    ok("head is at (0,1,0)", head == {"x": 0, "y": 1, "z": 0},
       f"head={head}")
    # Ghost should skip (0,0,0) and appear at (0,-1,0)
    ok("ghost skips occupied cell to (0,-1,0)",
       ghost == {"x": 0, "y": -1, "z": 0},
       f"ghost={ghost}")

    ss(c, "09a_ghost_before_intersection")

    # Place the intersection block
    r = c.place_block()
    ok("place at intersection succeeds", r.get("ok") is True, str(r))
    state = c.get_state()
    # intersection adds 2 blocks: junction (shared) + new block on other side
    ok("nblocks grew by 2 (junction + beyond)", state.get("nblocks") == 10,
       f"nblocks={state.get('nblocks')}")
    # Head is now beyond the junction at (0,-1,0)
    head_after = state.get("head", {})
    ok("head advanced past junction to (0,-1,0)",
       head_after == {"x": 0, "y": -1, "z": 0},
       f"head={head_after}")

    ss(c, "09b_after_intersection_placed")

    # ------------------------------------------------------------------
    section("9c. Self-intersection delete (regression)")
    # ------------------------------------------------------------------
    # One delete atomically undoes the whole intersection placement (junction + beyond)
    r = c.delete_block()
    ok("atomic delete of intersection placement ok", r.get("ok") is True)
    state = c.get_state()
    ok("nblocks = 8 after atomic delete (both junction and beyond removed)",
       state.get("nblocks") == 8,
       f"nblocks={state.get('nblocks')}")

    # The grid must still contain (0,0,0) — the original first block
    r_nc = c.start_chain_at(0, 0, 0)
    ok("origin block still exists after delete (no gap)",
       r_nc.get("ok") is True, str(r_nc))

    # Clean up the extra chain we just created
    c.switch_chain(0)
    ss(c, "09c_after_self_intersect_delete")

    # ------------------------------------------------------------------
    section("10. Camera rotation (regression for NULL-model crash)")
    # ------------------------------------------------------------------

    # Record initial camera state
    state0 = c.get_state()
    r = c.move_camera("rotate_v", dx=0.0, dy=0.1)
    ok("rotate_v returns ok", r.get("ok") is True, str(r))
    ok("elevation changes after rotate_v",
       abs(r.get("elevation", 0) - state0.get("elevation", -30)) > 0.01,
       f"elevation={r.get('elevation')}")

    r2 = c.move_camera("rotate_h", dx=0.1, dy=0.0)
    ok("rotate_h returns ok", r2.get("ok") is True)
    ok("azimuth changes after rotate_h",
       abs(r2.get("azimuth", 0) - r.get("azimuth", 135)) > 0.01,
       f"azimuth={r2.get('azimuth')}")

    r3 = c.move_camera("zoom", dx=0.0, dy=0.1)
    ok("zoom returns ok", r3.get("ok") is True)
    ok("distance changes after zoom",
       abs(r3.get("distance", 3.0) - r2.get("distance", 3.0)) > 1e-6,
       f"distance={r3.get('distance')}")

    r4 = c.move_camera("move_h", dx=0.05, dy=0.0)
    ok("move_h returns ok", r4.get("ok") is True)

    r5 = c.move_camera("move_v", dx=0.0, dy=0.05)
    ok("move_v returns ok", r5.get("ok") is True)

    r6 = c.move_camera("bad_action", dx=0.0, dy=0.0)
    ok("bad action returns error", r6.get("ok") is False, str(r6))

    ss(c, "10_camera_rotated")

    # ------------------------------------------------------------------
    section("9h. Multi-link junction: two consecutive occupied cells")
    # ------------------------------------------------------------------
    # A single PlaceBlock press should jump over ALL consecutive occupied
    # cells (not just one) and land on the first free cell.
    #
    # Layout (+X direction):
    #   (0,0,0) [start] → (1,0,0) → (2,0,0) → (3,0,0) [existing chain 0]
    #   Chain 1 starts at (1,0,0), goes +Y: (1,1,0) → (2,1,0) → (3,1,0)
    #   Turn -X: → (2,1,0) [already there, turn at (3,1,0)]
    #   Wait — easier scenario: build a straight row then come back through 2 cells.
    #
    # Simpler: chain 0 = (0,0,0) → (1,0,0) → (2,0,0) → (3,0,0) → (4,0,0)
    #          chain 1 starts at (2,0,0), goes +Y: (2,1,0)
    #          turn -X: (1,1,0) → (0,1,0)
    #          turn -Y: pressing PlaceBlock should pass THROUGH (0,0,0) AND land
    #          at... wait, (0,0,0) is on chain 0's axis so axis is occupied.
    #
    # Let's use perpendicular axes so the chain CAN pass through:
    #   Chain 0 travels +X: occupies x-axis of each cell it passes through.
    #   Chain 1 travels -Y through two of those cells: -Y axis is free in them.
    #   Chain 1: start at (1,2,0), dir -Y.
    #   PlaceBlock → (1,1,0) [free] → (1,0,0) [free] → places at (1,-1,0)... hmm
    #   that's normal, not a double junction.
    #
    # Correct double-junction: chain 1 travels -Y and hits TWO chain-0 cells.
    # Chain 0 goes +X: (0,0,0)→(1,0,0)→(2,0,0).  Also +X: (0,1,0)→(1,1,0)→(2,1,0).
    # Both rows are on Y=0 and Y=1, travelling in +X.
    # Chain 1 starts at (1,2,0), travels -Y.
    # (1,1,0) and (1,0,0) are both on chain 0 (+X axis occupied, -Y axis is free).
    # So chain 1 PlaceBlock → passes through (1,1,0) and (1,0,0) → lands at (1,-1,0).
    c.reset()
    c.set_direction("+X")
    c.place_block()   # (1,0,0)
    c.place_block()   # (2,0,0)
    c.set_direction("+Y")
    c.place_block()   # (2,1,0)
    c.set_direction("-X")
    c.place_block()   # (1,1,0)
    c.place_block()   # (0,1,0)
    c.set_direction("-Y")
    c.place_block()   # crosses through (0,0,0) → places at (0,-1,0)  [1 junction]

    state = c.get_state()
    ok("9h: after single junction, 1 chain with 8 blocks",
       state.get("nblocks") == 8,
       f"nblocks={state.get('nblocks')}")

    # Now build a double-junction scenario: two rows side by side, chain loops through both.
    # Chain 0 row at Y=0: (0,0,0)→(1,0,0)→(2,0,0)  (+X axis occupied)
    # Chain 0 row at Y=1: (0,1,0)→(1,1,0)→(2,1,0)  (+X axis occupied; via turns)
    # Chain 1 starts at (1,2,0), dir -Y → hits (1,1,0) [junction] then (1,0,0) [junction]
    # → places at (1,-1,0) via a single PlaceBlock (double junction skip).
    c.reset()
    c.set_direction("+X")
    c.place_block()   # (0,0,0) → (1,0,0)
    c.place_block()   # → (2,0,0)
    c.set_direction("+Y")
    c.place_block()   # → (2,1,0)
    c.set_direction("-X")
    c.place_block()   # → (1,1,0)
    c.place_block()   # → (0,1,0)
    c.set_direction("+Y")
    c.place_block()   # → (0,2,0)
    c.set_direction("+X")
    c.place_block()   # → (1,2,0)

    # Start chain 1 at (1,1,0), go -Y
    r = c.start_chain_at(1, 1, 0)
    ok("9h: start_chain_at(1,1,0) ok", r.get("ok") is True, str(r))
    c.set_direction("-Y")
    # PlaceBlock should pass through (1,0,0) [already on chain 0 X-axis] and land at (1,-1,0)
    r = c.place_block()
    ok("9h: single-press double-junction succeeds", r.get("ok") is True, str(r))

    state = c.get_state()
    head = state.get("head")
    ok("9h: head landed beyond double-junction at y=-1",
       head is not None and head.get("y") == -1,
       f"head={head}")

    # ------------------------------------------------------------------
    section("9i. Multi-junction atomic undo")
    # ------------------------------------------------------------------
    # One delete should undo the entire multi-junction PlaceBlock atomically
    nblocks_before = state.get("nblocks", 0)
    r = c.delete_block()
    ok("9i: delete after multi-junction ok", r.get("ok") is True, str(r))
    state_after = c.get_state()
    nblocks_after = state_after.get("nblocks", 0)
    # Multi-junction PlaceBlock added 2 blocks (1 junction + new_bead), so delete removes 2
    ok("9i: atomic delete removed all multi-junction blocks",
       nblocks_before - nblocks_after == 2,
       f"before={nblocks_before}, after={nblocks_after}")
    head_after = state_after.get("head")
    ok("9i: head returned to chain-1 start (1,1,0) after atomic undo",
       head_after == {"x": 1, "y": 1, "z": 0},
       f"head={head_after}")

    # ------------------------------------------------------------------
    section("9j. Multi-chain create and delete")
    # ------------------------------------------------------------------
    # Switch to chain 0 and verify it still works
    c.switch_chain(0)
    state = c.get_state()
    ok("9j: chain 0 active after switch", state.get("active_chain") == 0,
       f"active={state.get('active_chain')}")

    # Place a block on chain 0
    nblocks_c0_before = state.get("nblocks", 0)
    c.set_direction("+X")
    r_place = c.place_block()
    ok("9j: chain 0 place ok", r_place.get("ok") is True, str(r_place))
    state = c.get_state()
    ok("9j: nblocks grew by 1 on chain 0",
       state.get("nblocks", 0) == nblocks_c0_before + 1,
       f"before={nblocks_c0_before}, after={state.get('nblocks')}")

    # Delete it back
    r_del = c.delete_block()
    ok("9j: chain 0 delete ok", r_del.get("ok") is True, str(r_del))
    state = c.get_state()
    ok("9j: nblocks back to original after delete",
       state.get("nblocks", 0) == nblocks_c0_before,
       f"expected={nblocks_c0_before}, got={state.get('nblocks')}")

    # ------------------------------------------------------------------
    section("11. Screenshot")
    # ------------------------------------------------------------------

    ss_path = os.path.join(SCREENSHOT_DIR, "11_final_screenshot.ppm")
    r = c.screenshot(ss_path)
    ok("screenshot returns ok", r.get("ok") is True, str(r))
    ok("screenshot file exists", os.path.exists(ss_path))
    ok("screenshot has dimensions", r.get("width", 0) > 0 and
                                    r.get("height", 0) > 0,
       f"{r.get('width')}x{r.get('height')}")

    # ------------------------------------------------------------------
    section("12. Quit")
    # ------------------------------------------------------------------

    r = c.quit()
    ok("quit returns ok", r.get("ok") is True)
    time.sleep(1.5)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

print(f"\n{'='*60}")
print(f"  TEST SUMMARY")
print(f"{'='*60}")
print(f"  PASSED : {len(passed)}")
print(f"  FAILED : {len(failed)}")
print(f"  TOTAL  : {len(passed) + len(failed)}")
if failed:
    print(f"\n  Failed tests:")
    for f in failed:
        print(f"    - {f}")
print(f"\n  Screenshots: {SCREENSHOT_DIR}")
print()

sys.exit(0 if not failed else 1)
