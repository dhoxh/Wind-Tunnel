# RustFlow — Roadmap

**Goal:** a real-time GPU wind tunnel in Rust, plus a neural surrogate trained on its output.

This is a **learning / portfolio project**. Physical accuracy is explicitly not the bar — a
clean, demonstrable end-to-end loop is. See [Scope & honesty](#scope--honesty) for what that
means in practice.

---

## Start here

**Next action: the A0 spike — one trivial GPU compute dispatch, verified.**

Write a constant into a buffer from a compute shader, read it back on the CPU, assert the
value. No physics. Nothing else.

Why this and nothing else first: the fluid math is already solved and working on CPU. The
actual risk in Stage A is *Bevy/wgpu integration* — render-graph plumbing, buffer lifetimes,
async readback. Proving that on a trivial payload costs an hour or two and de-risks the
largest stage in the project. Everything below is blocked behind it.

Do not start porting the solver until the spike round-trips a value.

---

## What we have right now

Branch `claude/elegant-mccarthy-O0jYG` @ `fa363bf`. ~1,900 lines of Rust across 5 modules.

| Module | Lines | Responsibility |
|---|---|---|
| `fluid.rs` | 544 | Navier–Stokes solver |
| `geometry.rs` | 444 | Model import, placement, voxelization |
| `main.rs` | 527 | App wiring, settings UI, camera |
| `renderer.rs` | 233 | Streamline tracing + drawing |
| `grid.rs` | 188 | Adaptive domain, coordinate mapping |

### Solver (`fluid.rs`)

A collocated-grid **Stam "Stable Fluids"** implementation. Per step:

1. **Inflow** forcing on the upwind face (+X).
2. **Viscous diffusion** — Jacobi relaxation, skipped when viscosity is 0.
3. **Semi-Lagrangian advection** — unconditionally stable, first-order accurate.
4. **Turbulence** — random velocity kicks at cells adjacent to the body, plus vorticity
   confinement.
5. **Pressure projection** — 28 parallel Jacobi sweeps solving `∇²p = ∇·u`, then subtract
   `∇p`. This is the Helmholtz–Hodge decomposition: project the velocity field onto its
   divergence-free component.
6. **No-slip** — zero velocity inside solid cells; solids act as Neumann walls in the solve.

Parallelized across Bevy's `ComputeTaskPool` (advection, pressure, streamline tracing).
Pressure buffers ping-pong and warm-start from the previous frame.

### Domain (`grid.rs`)

Adaptive. When an object is placed, the grid **resizes and repositions to wrap it**: cell
size is tied to object height (~22 cells tall), per-axis cell counts follow the region's
shape, total cells capped at ~90k. The solver is skipped entirely when the domain is empty.

### Geometry (`geometry.rs`)

GLB/glTF import via `rfd` (multi-file `.gltf` copies its whole folder). Auto-fit to a fixed
target size, rotated to face the wind, seated at ride height. Surface-voxelized into the
solid mask with a 1-cell underbody carve. Ride height and heading re-voxelize live.

### Visualization (`renderer.rs`)

Streamlines traced through the velocity field by fixed arc-length steps, seeded as a tight
rake around the object's frontal area. Colored by speed, reversed so stagnation reads red.
Adaptive domain drawn as a wireframe box.

### UI & app (`main.rs`)

egui settings panel: wind speed (0–103 m/s ≈ 230 mph, with mph readout), viscosity,
turbulence, ride height, heading, streamline density, FPS cap, and visibility toggles. VSync
off with a frame limiter. Fly camera. Name-heuristic wheel spin.

### Housekeeping

`cargo audit`: 596 crates, **0 vulnerabilities** (one unmaintained-crate warning, `paste`,
transitively via wgpu — not ours to fix).

---

## Known gaps

Ordered by how much they block the ML work.

1. **No force computation.** Nothing computes drag or lift. The tunnel emits pictures, not
   numbers — so there is currently *nothing to train on*. This is the hard blocker for
   Stages C–E.
2. **Underbody flow is unverified.** Leading hypothesis: at fitted resolution the ride-height
   gap spans **less than one cell**, so voxelization welds the car to the floor. The current
   `UNDERBODY_CLEAR = 1` carve is a band-aid, not a fix. Needs instrumenting before guessing
   further.
3. **Surface voxelization, not SDF.** The car is a hollow shell of marked cells — staircased
   boundaries, potential leaks at coarse resolution, and a point-test `is_solid`.
4. **Static floor.** No rolling road. A stationary ground grows a boundary layer that
   corrupts exactly the underbody region we care about.
5. **Sim runs at render rate.** No fixed timestep, so rendering at 200 FPS solves at 200 FPS
   for zero visual benefit.
6. **Non-deterministic.** Turbulence injects unseeded RNG. Fine visually, fatal for training
   labels.
7. **No headless mode.** Required for dataset generation.
8. **Turbulence is ad-hoc.** Noise injection + vorticity confinement is a visual model, not a
   physical closure.
9. **Wheel spin is name-based.** Fails on models with generic node names.

---

## Roadmap

### Stage A — GPU solver

The largest and riskiest stage. Chosen deliberately: "3D Navier–Stokes compute solver in
Rust/wgpu" is a stronger artifact than tuned CPU loops, and it's what makes generating
thousands of training samples feasible.

- **A0. Spike.** Minimal compute dispatch + readback, verified. *(See [Start here](#start-here).)*
  - Decide en route: hook Bevy's render graph (idiomatic, more boilerplate, version-fragile)
    vs. grab `RenderDevice` and drive wgpu directly (simpler to reason about and debug).
    **Leaning: direct wgpu** — the solver isn't really part of the frame's render pipeline.
- **A1. Data layout.** **3D textures for velocity** — hardware trilinear filtering makes
  semi-Lagrangian advection a free sample, which is a real win over the CPU path. Pressure,
  divergence, and the solid mask as storage buffers. Ping-pong sampled ↔ storage.
- **A2. Port passes one at a time**, diffing each against the CPU solver: inflow → advect →
  curl/confinement → divergence → pressure → gradient subtract → obstacles.
  - **Keep the CPU solver permanently.** It is the correctness oracle (compute shaders have
    no printf; a numeric diff is the debugger), the fallback, and the benchmark baseline.
- **A3. Optimize the pressure loop.** 28 sequential dispatches of small kernels can become
  dispatch-overhead-bound rather than math-bound. Two standard fixes:
  - **Red-black Gauss–Seidel** — roughly halves iterations for equal residual.
  - **Batch k iterations per dispatch** via workgroup shared memory over a tile with a halo.
  - If it's still the bottleneck, the principled answer is a **multigrid V-cycle**. Jacobi and
    GS damp high-frequency error quickly but converge on low-frequency modes at a rate that
    degrades as `h → 0`; multigrid handles every frequency band and is ~O(n).
- **A4. Benchmark CPU vs GPU** and record the table. This is a deliverable, not just a number.

*Cheap win to grab en route:* move the sim to `FixedUpdate` (30–60 Hz). ~10 lines.

### Stage B — Make it measure

- **B1. Force integration.** Integrate `p · n` over the body surface, non-dimensionalize to
  Cd/Cl, display live. Simplest first version: read the pressure field back once per
  converged sample and integrate on CPU. **Design the readback path here** — Stage C depends
  on it.
- **B2. Switch geometry to an SDF.** Fixes the boundary quality *and* provides the canonical
  input encoding for a geometry-conditioned network. One change, two payoffs.
- **B3. Diagnose the underbody.** Log ride height *in cells*. If < 2, constrain the grid fit
  so the gap always resolves across 2–3 cells. Verify with a vertical slice view.
- **B4. Rolling road.** Floor moving at freestream speed. Cheap, and physically correct for
  automotive.

### Stage C — Data pipeline

Standard ETL shape: extract (run sim) → transform (normalize, non-dimensionalize) → load
(sample store). The unusual part is that extraction is expensive, so it must be batched,
deterministic, and resumable.

- **C1. Headless mode.** No render, no window. Easy and essential.
- **C2. Determinism.** Seed the turbulence RNG fixedly, or time-average to steady state.
  Noisy labels put a floor under achievable test error.
- **C3. Convergence criterion.** Run until the force residual stabilizes — not a fixed frame
  count, which either wastes compute or samples an unconverged field.
- **C4. Shape variation.** The hard part: learning geometry → aero needs many geometries.
  **Free-form deformation lattice** over one base model gives a continuous, samplable design
  space. (Alternative: parametric morphs — roofline, wing angle, diffuser, length.)
- **C5. Sampling.** Sobol or Latin hypercube over shape params × ride height × yaw × speed.
  Quasi-random sequences have lower discrepancy than uniform random, so coverage is better
  per sample — which matters when each sample costs seconds.
- **C6. Schema & storage.** Per sample: SDF, conditions, forces, optionally a downsampled
  field. Manifest in Parquet, arrays in HDF5 or `.npz`. Budget: ~1.4 MB/sample if storing
  full fields; **1–2k samples is plenty** to demonstrate the loop.

### Stage D — Train

- **D1. Scalars first.** SDF + conditions → Cd/Cl, small 3D CNN or PointNet. Trains in
  minutes and validates the whole pipeline before investing in anything larger.
- **D2. Field surrogate.** 3D U-Net, SDF → velocity + pressure. This is what enables the demo.
- **D3. Rigor.** Two mistakes reviewers look for:
  - **Split by geometry, not by sample.** The same car at different yaw in both train and
    test is leakage, and it inflates scores badly.
  - **Baseline first.** Predict-the-mean, then linear regression, *then* claim the network
    helped. Report the deltas.

PyTorch in Python — Rust's training ecosystem isn't there yet. Inference comes back to Rust
in Stage E.

### Stage E — Deploy back into the app

- **E1.** Export ONNX, run inference in Rust via `ort`.
- **E2.** **The demo:** toggle between solver-truth and NN-prediction side by side, with
  timings — *"solver: 3.2 s to converge / network: 8 ms."* That single screenshot carries the
  whole project.

---

## Decisions log

| Decision | Choice | Reasoning |
|---|---|---|
| Project goal | Learning / portfolio | Optimize for a clean demonstrable loop, not physical accuracy |
| First major work | GPU solver, skip incremental CPU tuning | Stronger artifact; unlocks feasible dataset generation |
| CPU solver | Keep permanently | Correctness oracle, fallback, benchmark baseline |
| Bevy integration | Leaning direct wgpu over render graph | Simpler to reason about and debug; solver isn't part of frame rendering |
| Velocity storage | 3D textures | Free hardware trilinear filtering for semi-Lagrangian advection |
| Geometry encoding | SDF | Better boundaries for the solver *and* canonical NN input |
| Dataset size | 1–2k samples | Sufficient to demonstrate the loop; avoids over-collecting |

---

## Scope & honesty

A network trained on this data learns **our solver, not reality**. The solver is:

- **First-order** semi-Lagrangian → strongly numerically diffusive
- **~90k cells** — real automotive CFD uses 10⁷–10⁸
- **No turbulence closure** — noise injection and vorticity confinement are visual devices
- **Staircased voxel walls**, no resolved boundary layer

A surrogate of one's own solver is a legitimate and widely-used technique, and the speedup is
real. But it will not produce trustworthy real-world Cd, and the writeup should say so
plainly. Stating the limitation precisely is a strength, not a weakness.

If real-world accuracy ever becomes the goal, the path is different: train on public
automotive CFD datasets (DrivAerNet++, AhmedML, DrivAerML) and use this tunnel as the
visualizer rather than the data source.

---

## Open questions

- Render graph vs. direct wgpu — resolve during A0.
- Base model for the FFD lattice in C4 — the BMW M4 GT3, or something simpler and cleaner
  like an Ahmed body? An Ahmed body has published reference values, which would let us sanity
  check Cd even though accuracy isn't the goal.
- Whether to time-average unsteady wakes or force a steady solution for C2.
