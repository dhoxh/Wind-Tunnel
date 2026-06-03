//! Grid-based incompressible Navier–Stokes solver (Jos Stam "Stable Fluids").
//!
//! This is a collocated 3D velocity field stored on the same lattice as
//! [`crate::grid::Grid3D`]. Each step performs:
//!   1. inflow forcing on the upwind (`x = 0`) face,
//!   2. semi-Lagrangian advection of velocity,
//!   3. optional vorticity-confinement turbulence,
//!   4. pressure projection (Jacobi) to enforce incompressibility,
//!   5. obstacle handling: zero velocity inside solid cells (no-slip).
//!
//! The per-cell loops (advection and the pressure iterations, which dominate
//! cost) are parallelized across Bevy's `ComputeTaskPool`. The pressure solve
//! uses Jacobi relaxation with two ping-ponged buffers so each sweep is
//! embarrassingly parallel and warm-starts from the previous frame.

use bevy::prelude::*;
use bevy::tasks::ComputeTaskPool;

use crate::grid::Grid3D;

/// Jacobi pressure iterations per projection.
const PRESSURE_ITERS: usize = 28;

/// All mutable fluid state for the domain. Velocity is stored in cell units
/// per second; one cell == `grid::CELL_SIZE` world units.
#[derive(Resource)]
pub struct FluidSim {
    pub w: usize,
    pub h: usize,
    pub d: usize,

    // Current velocity components.
    pub u: Vec<f32>,
    pub v: Vec<f32>,
    pub ws: Vec<f32>,

    // Scratch buffers reused across steps to avoid per-frame allocation.
    u0: Vec<f32>,
    v0: Vec<f32>,
    w0: Vec<f32>,
    p: Vec<f32>,
    p1: Vec<f32>,
    div: Vec<f32>,

    // User-tunable parameters.
    pub wind_speed: f32,
    pub viscosity: f32,
    /// Vorticity-confinement strength — injects swirling turbulence, most
    /// visible in the wake behind the obstacle.
    pub turbulence: f32,
    pub running: bool,
}

impl FluidSim {
    pub fn new(w: usize, h: usize, d: usize) -> Self {
        let n = w * h * d;
        Self {
            w,
            h,
            d,
            u: vec![0.0; n],
            v: vec![0.0; n],
            ws: vec![0.0; n],
            u0: vec![0.0; n],
            v0: vec![0.0; n],
            w0: vec![0.0; n],
            p: vec![0.0; n],
            p1: vec![0.0; n],
            div: vec![0.0; n],
            wind_speed: 20.0,
            viscosity: 0.0,
            turbulence: 1.0,
            running: true,
        }
    }

    #[inline]
    fn idx(&self, x: usize, y: usize, z: usize) -> usize {
        z * self.w * self.h + y * self.w + x
    }

    /// Reset the entire velocity field to rest.
    pub fn reset(&mut self) {
        for buf in [
            &mut self.u,
            &mut self.v,
            &mut self.ws,
            &mut self.u0,
            &mut self.v0,
            &mut self.w0,
            &mut self.p,
            &mut self.p1,
            &mut self.div,
        ] {
            buf.iter_mut().for_each(|c| *c = 0.0);
        }
    }

    /// Trilinearly sample the velocity field at continuous grid coordinates.
    /// Used by the streamline tracer. Out-of-domain samples clamp to the edge.
    pub fn sample_velocity(&self, gx: f32, gy: f32, gz: f32) -> Vec3 {
        let cx = gx.clamp(0.0, self.w as f32 - 1.001);
        let cy = gy.clamp(0.0, self.h as f32 - 1.001);
        let cz = gz.clamp(0.0, self.d as f32 - 1.001);

        let x0 = cx.floor() as usize;
        let y0 = cy.floor() as usize;
        let z0 = cz.floor() as usize;
        let x1 = (x0 + 1).min(self.w - 1);
        let y1 = (y0 + 1).min(self.h - 1);
        let z1 = (z0 + 1).min(self.d - 1);

        let fx = cx - x0 as f32;
        let fy = cy - y0 as f32;
        let fz = cz - z0 as f32;

        let lerp = |a: f32, b: f32, t: f32| a + (b - a) * t;
        let trilerp = |field: &[f32]| {
            let c000 = field[self.idx(x0, y0, z0)];
            let c100 = field[self.idx(x1, y0, z0)];
            let c010 = field[self.idx(x0, y1, z0)];
            let c110 = field[self.idx(x1, y1, z0)];
            let c001 = field[self.idx(x0, y0, z1)];
            let c101 = field[self.idx(x1, y0, z1)];
            let c011 = field[self.idx(x0, y1, z1)];
            let c111 = field[self.idx(x1, y1, z1)];

            let c00 = lerp(c000, c100, fx);
            let c10 = lerp(c010, c110, fx);
            let c01 = lerp(c001, c101, fx);
            let c11 = lerp(c011, c111, fx);
            let c0 = lerp(c00, c10, fy);
            let c1 = lerp(c01, c11, fy);
            lerp(c0, c1, fz)
        };

        Vec3::new(trilerp(&self.u), trilerp(&self.v), trilerp(&self.ws))
    }

    /// Advance the simulation by `dt` seconds against the obstacle mask in `grid`.
    pub fn step(&mut self, grid: &Grid3D, dt: f32) {
        if !self.running {
            return;
        }
        // Clamp dt so a hitching frame can't blow the advection backtrace out
        // of the domain.
        let dt = dt.min(1.0 / 30.0);

        self.apply_inflow();
        self.enforce_obstacles(grid);

        // --- Diffusion (viscosity) ------------------------------------------
        if self.viscosity > 0.0 {
            let a = dt * self.viscosity * (self.w * self.h * self.d) as f32 * 0.0005;
            self.u0.copy_from_slice(&self.u);
            self.v0.copy_from_slice(&self.v);
            self.w0.copy_from_slice(&self.ws);
            Self::diffuse(self.w, self.h, self.d, &mut self.u, &self.u0, a, grid);
            Self::diffuse(self.w, self.h, self.d, &mut self.v, &self.v0, a, grid);
            Self::diffuse(self.w, self.h, self.d, &mut self.ws, &self.w0, a, grid);
            self.enforce_obstacles(grid);
        }

        // --- Advection (parallel) -------------------------------------------
        self.u0.copy_from_slice(&self.u);
        self.v0.copy_from_slice(&self.v);
        self.w0.copy_from_slice(&self.ws);
        let (w, h, d) = (self.w, self.h, self.d);
        {
            let (vu, vv, vw, solid) = (&self.u0, &self.v0, &self.w0, &grid.solid);
            par_fill(&mut self.u, |i| advect_cell(i, w, h, d, vu, vu, vv, vw, dt, solid));
        }
        {
            let (vu, vv, vw, solid) = (&self.u0, &self.v0, &self.w0, &grid.solid);
            par_fill(&mut self.v, |i| advect_cell(i, w, h, d, vv, vu, vv, vw, dt, solid));
        }
        {
            let (vu, vv, vw, solid) = (&self.u0, &self.v0, &self.w0, &grid.solid);
            par_fill(&mut self.ws, |i| advect_cell(i, w, h, d, vw, vu, vv, vw, dt, solid));
        }

        self.apply_inflow();
        self.enforce_obstacles(grid);

        // --- Turbulence force (folded into the single projection below) -----
        if self.turbulence > 0.0 {
            self.apply_turbulence(grid, dt);
            self.enforce_obstacles(grid);
        }

        self.project(grid);
        self.apply_inflow();
        self.enforce_obstacles(grid);
    }

    /// Drive the upwind face at the chosen wind speed, in the +X direction.
    fn apply_inflow(&mut self) {
        let speed_cells = self.wind_speed / crate::grid::CELL_SIZE;
        let xmax = 2.min(self.w);
        for z in 0..self.d {
            for y in 0..self.h {
                for x in 0..xmax {
                    let i = self.idx(x, y, z);
                    self.u[i] = speed_cells;
                    self.v[i] = 0.0;
                    self.ws[i] = 0.0;
                }
            }
        }
    }

    /// No-slip obstacle boundary: zero velocity inside solid cells.
    fn enforce_obstacles(&mut self, grid: &Grid3D) {
        for i in 0..self.u.len() {
            if grid.solid[i] {
                self.u[i] = 0.0;
                self.v[i] = 0.0;
                self.ws[i] = 0.0;
            }
        }
    }

    /// Viscous diffusion via Jacobi relaxation (only runs when viscosity > 0).
    fn diffuse(w: usize, h: usize, d: usize, field: &mut [f32], q0: &[f32], a: f32, grid: &Grid3D) {
        if w < 3 || h < 3 || d < 3 {
            return;
        }
        let idx = |x: usize, y: usize, z: usize| z * w * h + y * w + x;
        for _ in 0..12 {
            for z in 1..d - 1 {
                for y in 1..h - 1 {
                    for x in 1..w - 1 {
                        let i = idx(x, y, z);
                        if grid.solid[i] {
                            continue;
                        }
                        let nb = |nx: usize, ny: usize, nz: usize| {
                            let j = idx(nx, ny, nz);
                            if grid.solid[j] { field[i] } else { field[j] }
                        };
                        let sum = nb(x + 1, y, z)
                            + nb(x - 1, y, z)
                            + nb(x, y + 1, z)
                            + nb(x, y - 1, z)
                            + nb(x, y, z + 1)
                            + nb(x, y, z - 1);
                        field[i] = (q0[i] + a * sum) / (1.0 + 6.0 * a);
                    }
                }
            }
        }
    }

    /// Vorticity confinement: find each cell's local swirl (curl) and push
    /// energy back into it. Scratch lives in the advection buffers
    /// (`u0`/`v0`/`w0` hold the curl, `div` its magnitude); the subsequent
    /// projection re-derives `div`.
    fn apply_turbulence(&mut self, grid: &Grid3D, dt: f32) {
        let (w, h, d) = (self.w, self.h, self.d);
        if w < 3 || h < 3 || d < 3 {
            return;
        }
        let idx = |x: usize, y: usize, z: usize| z * w * h + y * w + x;

        for z in 1..d - 1 {
            for y in 1..h - 1 {
                for x in 1..w - 1 {
                    let i = idx(x, y, z);
                    if grid.solid[i] {
                        self.u0[i] = 0.0;
                        self.v0[i] = 0.0;
                        self.w0[i] = 0.0;
                        self.div[i] = 0.0;
                        continue;
                    }
                    let cx = (self.ws[idx(x, y + 1, z)] - self.ws[idx(x, y - 1, z)]
                        - (self.v[idx(x, y, z + 1)] - self.v[idx(x, y, z - 1)]))
                        * 0.5;
                    let cy = (self.u[idx(x, y, z + 1)] - self.u[idx(x, y, z - 1)]
                        - (self.ws[idx(x + 1, y, z)] - self.ws[idx(x - 1, y, z)]))
                        * 0.5;
                    let cz = (self.v[idx(x + 1, y, z)] - self.v[idx(x - 1, y, z)]
                        - (self.u[idx(x, y + 1, z)] - self.u[idx(x, y - 1, z)]))
                        * 0.5;
                    self.u0[i] = cx;
                    self.v0[i] = cy;
                    self.w0[i] = cz;
                    self.div[i] = (cx * cx + cy * cy + cz * cz).sqrt();
                }
            }
        }

        let eps = self.turbulence;
        for z in 1..d - 1 {
            for y in 1..h - 1 {
                for x in 1..w - 1 {
                    let i = idx(x, y, z);
                    if grid.solid[i] {
                        continue;
                    }
                    let gx = (self.div[idx(x + 1, y, z)] - self.div[idx(x - 1, y, z)]) * 0.5;
                    let gy = (self.div[idx(x, y + 1, z)] - self.div[idx(x, y - 1, z)]) * 0.5;
                    let gz = (self.div[idx(x, y, z + 1)] - self.div[idx(x, y, z - 1)]) * 0.5;
                    let mag = (gx * gx + gy * gy + gz * gz).sqrt() + 1e-5;
                    let (nx, ny, nz) = (gx / mag, gy / mag, gz / mag);

                    let (cx, cy, cz) = (self.u0[i], self.v0[i], self.w0[i]);
                    self.u[i] += eps * (ny * cz - nz * cy) * dt;
                    self.v[i] += eps * (nz * cx - nx * cz) * dt;
                    self.ws[i] += eps * (nx * cy - ny * cx) * dt;
                }
            }
        }
    }

    /// Pressure projection via parallel Jacobi relaxation. Solid cells act as
    /// Neumann walls so the flow is forced around obstacles. `p` warm-starts
    /// from the previous frame.
    fn project(&mut self, grid: &Grid3D) {
        let (w, h, d) = (self.w, self.h, self.d);
        if w < 3 || h < 3 || d < 3 {
            return;
        }
        let idx = |x: usize, y: usize, z: usize| z * w * h + y * w + x;

        // Divergence (serial single pass).
        for z in 1..d - 1 {
            for y in 1..h - 1 {
                for x in 1..w - 1 {
                    let i = idx(x, y, z);
                    if grid.solid[i] {
                        self.div[i] = 0.0;
                        continue;
                    }
                    self.div[i] = -0.5
                        * ((self.u[idx(x + 1, y, z)] - self.u[idx(x - 1, y, z)])
                            + (self.v[idx(x, y + 1, z)] - self.v[idx(x, y - 1, z)])
                            + (self.ws[idx(x, y, z + 1)] - self.ws[idx(x, y, z - 1)]));
                }
            }
        }

        // Jacobi iterations (parallel), ping-ponging p <-> p1.
        for _ in 0..PRESSURE_ITERS {
            {
                let (p_old, div, solid) = (&self.p, &self.div, &grid.solid);
                par_fill(&mut self.p1, |i| jacobi_cell(i, w, h, d, p_old, div, solid));
            }
            std::mem::swap(&mut self.p, &mut self.p1);
        }

        // Subtract the pressure gradient (serial single pass).
        for z in 1..d - 1 {
            for y in 1..h - 1 {
                for x in 1..w - 1 {
                    let i = idx(x, y, z);
                    if grid.solid[i] {
                        continue;
                    }
                    let px = |nx: usize, ny: usize, nz: usize| {
                        let j = idx(nx, ny, nz);
                        if grid.solid[j] { self.p[i] } else { self.p[j] }
                    };
                    self.u[i] -= 0.5 * (px(x + 1, y, z) - px(x - 1, y, z));
                    self.v[i] -= 0.5 * (px(x, y + 1, z) - px(x, y - 1, z));
                    self.ws[i] -= 0.5 * (px(x, y, z + 1) - px(x, y, z - 1));
                }
            }
        }
    }
}

/// Fill `out[i] = f(i)` for every cell, in parallel across the compute pool.
/// `f` reads from immutable input arrays only, so disjoint output chunks are
/// data-race free.
fn par_fill<F>(out: &mut [f32], f: F)
where
    F: Fn(usize) -> f32 + Sync,
{
    let pool = ComputeTaskPool::get();
    let threads = pool.thread_num().max(1);
    let chunk = out.len().div_ceil(threads).max(1);
    pool.scope(|scope| {
        for (ci, slice) in out.chunks_mut(chunk).enumerate() {
            let base = ci * chunk;
            let f = &f;
            scope.spawn(async move {
                for (k, cell) in slice.iter_mut().enumerate() {
                    *cell = f(base + k);
                }
            });
        }
    });
}

/// Semi-Lagrangian backtrace advection for one cell; boundary/solid cells keep
/// their previous value (`q0[i]`).
#[allow(clippy::too_many_arguments)]
#[inline]
fn advect_cell(
    i: usize,
    w: usize,
    h: usize,
    d: usize,
    q0: &[f32],
    u: &[f32],
    v: &[f32],
    ws: &[f32],
    dt: f32,
    solid: &[bool],
) -> f32 {
    let wh = w * h;
    let z = i / wh;
    let rem = i - z * wh;
    let y = rem / w;
    let x = rem - y * w;
    if x == 0 || y == 0 || z == 0 || x >= w - 1 || y >= h - 1 || z >= d - 1 || solid[i] {
        return q0[i];
    }

    let idx = |x: usize, y: usize, z: usize| z * wh + y * w + x;
    let mut px = x as f32 - dt * u[i];
    let mut py = y as f32 - dt * v[i];
    let mut pz = z as f32 - dt * ws[i];
    px = px.clamp(0.5, w as f32 - 1.5);
    py = py.clamp(0.5, h as f32 - 1.5);
    pz = pz.clamp(0.5, d as f32 - 1.5);

    let x0 = px.floor() as usize;
    let y0 = py.floor() as usize;
    let z0 = pz.floor() as usize;
    let (x1, y1, z1) = (x0 + 1, y0 + 1, z0 + 1);
    let sx = px - x0 as f32;
    let sy = py - y0 as f32;
    let sz = pz - z0 as f32;

    let l = |a: f32, b: f32, t: f32| a + (b - a) * t;
    let c00 = l(q0[idx(x0, y0, z0)], q0[idx(x1, y0, z0)], sx);
    let c10 = l(q0[idx(x0, y1, z0)], q0[idx(x1, y1, z0)], sx);
    let c01 = l(q0[idx(x0, y0, z1)], q0[idx(x1, y0, z1)], sx);
    let c11 = l(q0[idx(x0, y1, z1)], q0[idx(x1, y1, z1)], sx);
    let c0 = l(c00, c10, sy);
    let c1 = l(c01, c11, sy);
    l(c0, c1, sz)
}

/// One Jacobi pressure update; boundary/solid cells keep `p[i]`.
#[inline]
fn jacobi_cell(i: usize, w: usize, h: usize, d: usize, p: &[f32], div: &[f32], solid: &[bool]) -> f32 {
    let wh = w * h;
    let z = i / wh;
    let rem = i - z * wh;
    let y = rem / w;
    let x = rem - y * w;
    if x == 0 || y == 0 || z == 0 || x >= w - 1 || y >= h - 1 || z >= d - 1 || solid[i] {
        return p[i];
    }
    let me = p[i];
    let idx = |x: usize, y: usize, z: usize| z * wh + y * w + x;
    let nb = |nx: usize, ny: usize, nz: usize| {
        let j = idx(nx, ny, nz);
        if solid[j] { me } else { p[j] }
    };
    let sum = nb(x + 1, y, z)
        + nb(x - 1, y, z)
        + nb(x, y + 1, z)
        + nb(x, y - 1, z)
        + nb(x, y, z + 1)
        + nb(x, y, z - 1);
    (div[i] + sum) / 6.0
}
