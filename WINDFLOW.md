# RustFlow — Navier–Stokes Wind Flow

This document describes the wind-flow simulation that now ships in RustFlow:
what it does, how it works, and how to use it.

The goal is **realistic, interactive wind flow in Rust**: import an arbitrary
3D model (for example, something downloaded from Sketchfab), drop it into a
virtual wind tunnel, and watch the air stream, wrap, and form wakes around it —
with live toggles for speed, viscosity, and what you're looking at.

---

## What it does

- **Real-time incompressible flow.** A 3D grid-based Navier–Stokes solver runs
  every frame and pushes air in the +X direction through the tunnel.
- **Obstacle-aware.** Any solid geometry in the domain (the default block, or an
  imported model) becomes a no-slip obstacle. The flow is forced to go *around*
  it, producing the speed-up over the body and the low-speed wake behind it.
- **Model import from disk.** Press **O** to open a native file picker, choose a
  `.glb`/`.gltf` model, and RustFlow auto-fits it into the middle of the tunnel
  and voxelizes it into the solver so the wind reacts to its real shape.
- **Streamline visualization.** A regular lattice of seed points on the upwind
  face is traced through the velocity field into smooth streamlines — the way a
  real wind tunnel reveals flow with a smoke rake. Lines bend over and around
  the body and trail into the wake, colored by local speed (blue = slow → red =
  fast).
- **On-the-ground placement.** The obstacle sits on the tunnel floor at an
  adjustable **ride height** (default 40 mm, 20–150 mm), so an imported car
  rests at realistic ground clearance and you can study ground effect.
- **Settings menu.** An on-screen panel (egui) with sliders and toggles for wind
  speed, viscosity, ride height, model heading, streamline density, FPS cap, and
  every visualization layer — all live while the sim runs. VSync is off and the
  frame rate is governed by the FPS-cap slider.

---

## How it works

### Domain & grid (`src/grid.rs`)

The tunnel is a uniform 3D lattice (`GRID_WIDTH × GRID_HEIGHT × GRID_DEPTH`
cells of `CELL_SIZE` world units). `Grid3D` owns:

- `solid[]` — the obstacle mask (which cells are blocked),
- `density[]` — an optional scalar dye field,
- and the conversions between integer cell coordinates and Bevy world-space, so
  the solver, the obstacle meshes, and the particles all agree on geometry.

### The solver (`src/fluid.rs`)

A collocated-grid implementation of Jos Stam's **"Stable Fluids"** method. Each
step:

1. **Inflow** — the upwind face is driven at the chosen wind speed (+X).
2. **Diffusion** — viscous smoothing via Jacobi relaxation (skipped when
   viscosity is 0, so by default the flow is effectively inviscid and lively).
3. **Advection** — semi-Lagrangian backtrace: each cell pulls its new velocity
   from where the flow came from, which is unconditionally stable.
4. **Pressure projection** — a Gauss–Seidel solve removes divergence so the
   field is (approximately) incompressible. Solid cells act as Neumann walls,
   which is what makes the air wrap around obstacles instead of through them.
5. **Obstacle handling** — velocity inside solid cells is zeroed (no-slip).

This trades engineering-grade CFD accuracy for **stability and responsiveness**,
which is exactly what an interactive visualization needs.

### Model import & voxelization (`src/geometry.rs`)

1. **Pick** — `rfd` opens a native dialog filtered to `.glb`/`.gltf`. A single
   `.glb` is copied as-is; a multi-file `.gltf` brings its whole folder (the
   `.bin` buffers and textures) along.
2. **Auto-fit** — the combined bounding box is scaled to the tunnel using the
   model's *rotated* extents (so a car's long axis ends up along the flow).
3. **Face the wind** — the model is rotated to point into the oncoming airflow;
   the heading is adjustable with a slider if a model's forward axis differs.
4. **Seat on the ground** — the model is translated so its underside sits at the
   chosen ride height above the floor.
5. **Voxelize** — every triangle is surface-sampled into `Grid3D::solid`, so the
   solver sees the model's actual silhouette. Changing ride height or heading
   re-seats and re-voxelizes the model live.

### Visualization (`src/renderer.rs`)

Each frame, a `density × density` lattice of seed points on the inlet is traced
through the (trilinearly sampled) velocity field by fixed-arc-length steps,
producing evenly spaced, smooth streamlines drawn with Bevy gizmos. A trace
stops when it reaches the outlet, a wall, or the obstacle, and each segment is
colored by local speed.

---

## Controls

Most settings live in the **Wind Tunnel** panel (top-left): sliders for wind
speed, viscosity, ride height, heading, streamline density, and FPS cap, plus
toggles and import/restore buttons. Keyboard shortcuts mirror the common ones:

| Key | Action |
| --- | --- |
| `O` | Import a 3D model (GLB / glTF) via file picker |
| `X` | Clear the import and restore the default cube |
| `Enter` | Pause / resume the simulation |
| `R` | Reset the flow field to rest |
| `P` | Toggle streamlines |
| `B` | Toggle obstacle visibility |
| `C` | Toggle color-by-speed |
| `W A S D` | Move camera |
| `Space` / `Shift` | Move camera up / down |
| Hold Left-Mouse | Look around (ignored while over the settings panel) |

---

## Running

```bash
cargo run --release
```

> **Linux build note:** Bevy needs a few system dev packages
> (`libasound2-dev`, `libudev-dev`, `libxkbcommon-dev`). The manifest forces
> `wayland-sys` to load libwayland at runtime (`dlopen`) so no wayland dev
> package is required. macOS and Windows need nothing extra.

---

## Roadmap

- GPU compute solver (wgpu) for higher-resolution, full-3D grids.
- Solid-voxel fill (interior) in addition to surface voxelization.
- Additional visualization modes: pressure field, vorticity heatmaps, and
  arrow/streamline glyphs.
- Drag-and-drop import in addition to the file picker.
- Adjustable inflow direction and turbulence (vorticity confinement).
