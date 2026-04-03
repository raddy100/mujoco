"""
bench_cg.py  —  Targeted CG solver + sphere contact benchmark

Tests CG stability boundary, sphere-contact optimization, and
finds the best combination for the chainmaker use case.
"""

import math
import time
import mujoco

N = 40
BEAD = 0.05
HALF = BEAD / 2.0
GAP  = BEAD * 0.05
STEP = BEAD * 1.05
DAMP = 0.15
WARMUP = 200
MEAS   = 1000


def cone_deg():
    a = HALF
    c = 1.0 + (GAP / a)
    sq2 = math.sqrt(2.0)
    if c >= sq2:
        phi = math.pi / 2.0
    else:
        s = max(0.0, min(1.0, c / sq2))
        phi = math.asin(s) - math.pi / 4.0
    phi = max(math.radians(5.0), min(math.radians(80.0), phi))
    return math.degrees(phi) * 0.5


def build_xml(use_sphere_contact=False):
    cd = cone_deg()
    solref = "0.15 0.7"
    solimp = "0.9 0.95 0.001 0.5 2.0"
    lines = [
        '<mujoco model="b">',
        '  <option/>',
        '  <worldbody>',
        f'  <geom type="plane" size="25 25 0.1" contype="1" conaffinity="7" '
        f'rgba=".5 .5 .5 1" solref="{solref}" solimp="{solimp}"/>',
    ]
    for i in range(N):
        ind = "  " * (i + 2)
        if i == 0:
            lines.append(f'{ind}<body name="b{i}" pos="0 0 {BEAD}">')
            lines.append(f'{ind}  <joint type="free" damping="{DAMP}"/>')
        else:
            ax = -1.0 * (HALF + GAP * 0.5)
            lines.append(f'{ind}<body name="b{i}" pos="{STEP} 0 0">')
            lines.append(
                f'{ind}  <joint type="ball" damping="{DAMP}" '
                f'pos="{ax:.5f} 0 0" limited="true" range="0 {cd:.4f}"/>'
            )
        if use_sphere_contact:
            r = HALF * 0.85
            lines.append(
                f'{ind}  <geom type="sphere" size="{r:.5f}" condim="1" '
                f'contype="1" conaffinity="1" rgba=".2 .6 1 1" '
                f'solref="{solref}" solimp="{solimp}"/>'
            )
        else:
            lines.append(
                f'{ind}  <geom type="box" size="{HALF} {HALF} {HALF}" '
                f'condim="1" contype="1" conaffinity="1" rgba=".2 .6 1 1" '
                f'solref="{solref}" solimp="{solimp}"/>'
            )
    for i in range(N - 1, -1, -1):
        ind = "  " * (i + 2)
        lines.append(f"{ind}</body>")
    lines.append("  </worldbody>")
    lines.append("  <contact>")
    for i in range(N - 2):
        lines.append(f'    <exclude body1="b{i}" body2="b{i+2}"/>')
    lines.append("  </contact>")
    lines.append("</mujoco>")
    return "\n".join(lines)


newton = int(mujoco.mjtSolver.mjSOL_NEWTON)
cg     = int(mujoco.mjtSolver.mjSOL_CG)
ifast  = int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
sparse = int(mujoco.mjtJacobian.mjJAC_SPARSE)
nocon  = int(mujoco.mjtDisableBit.mjDSBL_CONTACT)

xml_box    = build_xml(False)
xml_sphere = build_xml(True)

configs = [
    # Baseline
    ("01 BASELINE Newton box dt=.002 i20",
     xml_box, dict(timestep=0.002, iterations=20, solver=newton,
                   integrator=ifast, jacobian=sparse)),
    # CG stability tests on box
    ("02 CG box dt=.002 i20",
     xml_box, dict(timestep=0.002, iterations=20, solver=cg,
                   integrator=ifast, jacobian=sparse)),
    ("03 CG box dt=.003 i15",
     xml_box, dict(timestep=0.003, iterations=15, solver=cg,
                   integrator=ifast, jacobian=sparse)),
    ("04 CG box dt=.004 i15",
     xml_box, dict(timestep=0.004, iterations=15, solver=cg,
                   integrator=ifast, jacobian=sparse)),
    ("05 CG box dt=.005 i15",
     xml_box, dict(timestep=0.005, iterations=15, solver=cg,
                   integrator=ifast, jacobian=sparse)),
    ("06 CG box dt=.005 i20",
     xml_box, dict(timestep=0.005, iterations=20, solver=cg,
                   integrator=ifast, jacobian=sparse)),
    ("07 CG box dt=.005 i10 tol=1e-5",
     xml_box, dict(timestep=0.005, iterations=10, tolerance=1e-5,
                   solver=cg, integrator=ifast, jacobian=sparse)),
    # Sphere contact (reduces contact points from 4 to 1 per block)
    ("08 Newton sphere dt=.002 i20",
     xml_sphere, dict(timestep=0.002, iterations=20, solver=newton,
                      integrator=ifast, jacobian=sparse)),
    ("09 Newton sphere dt=.005 i15",
     xml_sphere, dict(timestep=0.005, iterations=15, solver=newton,
                      integrator=ifast, jacobian=sparse)),
    ("10 CG sphere dt=.003 i15",
     xml_sphere, dict(timestep=0.003, iterations=15, solver=cg,
                      integrator=ifast, jacobian=sparse)),
    ("11 CG sphere dt=.005 i15",
     xml_sphere, dict(timestep=0.005, iterations=15, solver=cg,
                      integrator=ifast, jacobian=sparse)),
    # No contacts + fixed timestep (structural viz mode)
    ("12 Newton nocon dt=.005 i10",
     xml_box, dict(timestep=0.005, iterations=10, solver=newton,
                   integrator=ifast, jacobian=sparse, disableflags=nocon)),
    ("13 CG nocon dt=.005 i10",
     xml_box, dict(timestep=0.005, iterations=10, solver=cg,
                   integrator=ifast, jacobian=sparse, disableflags=nocon)),
]

hdr = f"{'Config':<40} {'ms/step':>9} {'ncon':>5} {'steps/frm':>10} {'ms/frm':>8} {'RT':>6}  {'Status'}"
print(hdr)
print("-" * 90)

baseline_ms = None
results = []

for label, xml, opts in configs:
    try:
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        for k, v in opts.items():
            setattr(m.opt, k, v)

        for _ in range(WARMUP):
            mujoco.mj_step(m, d)

        stable = True
        t0 = time.perf_counter()
        for _ in range(MEAS):
            mujoco.mj_step(m, d)
            if not (abs(d.qpos[0]) < 200):
                stable = False
                break
        wall_ms = (time.perf_counter() - t0) / MEAS * 1000.0

        dt_ms  = m.opt.timestep * 1000.0
        spf    = 16.67 / wall_ms
        mspf   = spf * dt_ms
        rt     = dt_ms / wall_ms
        ncon   = int(d.ncon)
        status = "OK" if stable else "UNSTABLE"

        if label.startswith("01"):
            baseline_ms = wall_ms
        speedup = (baseline_ms / wall_ms) if baseline_ms else 1.0
        results.append((label, wall_ms, rt, ncon, spf, mspf, status, speedup))

        flag = " ★" if (speedup > 4 and stable) else ""
        print(
            f"{label:<40} {wall_ms:>9.3f} {ncon:>5} {spf:>10.1f} "
            f"{mspf:>8.1f} {rt:>6.2f}x  {status}{flag}"
        )
    except Exception as e:
        print(f"{label:<40}  FAILED: {e}")

print("-" * 90)
print()
print("Effective speedup vs baseline (per 60Hz frame):")
print()
for label, wms, rt, ncon, spf, mspf, status, speedup in results:
    if status == "OK":
        print(f"  {label:<40}  {speedup:5.1f}x faster per step  (RT={rt:.2f}x)")
