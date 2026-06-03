//! Grid-based incompressible Navier–Stokes solver (Jos Stam "Stable Fluids").
//!
//! This is a collocated 3D velocity field stored on the same lattice as
//! [`crate::grid::Grid3D`]. Each step performs:
//!   1. inflow forcing on the upwind (`x = 0`) face,
//!   2. semi-Lagrangian advection of velocity,
//!   3. pressure projection (Gauss–Seidel) to enforce incompressibility,
//!   4. obstacle handling: zero velocity inside solid cells and a no-slip
//!      boundary against them.
//!
//! It trades engineering-grade accuracy for unconditional stability and
//! real-time interactivity, which is exactly what an interactive wind tunnel
//! visualization needs.

use bevy::prelude::*;

use crate::grid::Grid3D;

/// Iterations of the Gauss–Seidel pressure solve per step. Higher = more
/// incompressible (cleaner wrapping around obstacles) but more expensive.
const PRESSURE_ITERS: usize = 20;

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
            div: vec![0.0; n],
            wind_speed: 20.0,
            viscosity: 0.0,
            turbulence: 0.5,
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
            &mut self.div,
        ] {
            buf.iter_mut().for_each(|c| *c = 0.0);
        }
    }

    /// Trilinearly sample the velocity field at continuous grid coordinates.
    /// Used by the particle integrator. Out-of-domain samples clamp to the edge.
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
            self.project(grid);
        }

        // --- Advection -------------------------------------------------------
        self.u0.copy_from_slice(&self.u);
        self.v0.copy_from_slice(&self.v);
        self.w0.copy_from_slice(&self.ws);
        Self::advect(self.w, self.h, self.d, &mut self.u, &self.u0, &self.u0, &self.v0, &self.w0, dt, grid);
        Self::advect(self.w, self.h, self.d, &mut self.v, &self.v0, &self.u0, &self.v0, &self.w0, dt, grid);
        Self::advect(self.w, self.h, self.d, &mut self.ws, &self.w0, &self.u0, &self.v0, &self.w0, dt, grid);

        self.apply_inflow();
        self.enforce_obstacles(grid);
        self.project(grid);

        // --- Turbulence (vorticity confinement) -----------------------------
        if self.turbulence > 0.0 {
            self.apply_turbulence(grid, dt);
            self.enforce_obstacles(grid);
            self.project(grid);
        }

        self.apply_inflow();
        self.enforce_obstacles(grid);
    }

    /// Vorticity confinement: find each cell's local swirl (curl) and push
    /// energy back into it, sharpening and sustaining eddies that numerical
    /// diffusion would otherwise smear away. Scratch lives in the advection
    /// buffers (`u0`/`v0`/`w0` hold the curl, `div` its magnitude).
    fn apply_turbulence(&mut self, grid: &Grid3D, dt: f32) {
        let (w, h, d) = (self.w, self.h, self.d);
        if w < 3 || h < 3 || d < 3 {
            return;
        }
        let idx = |x: usize, y: usize, z: usize| z * w * h + y * w + x;

        // Curl of the velocity field.
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

        // Confinement force f = eps * (N x curl), N = normalized grad|curl|.
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
                    // f = N x curl
                    let fx = ny * cz - nz * cy;
                    let fy = nz * cx - nx * cz;
                    let fz = nx * cy - ny * cx;

                    self.u[i] += eps * fx * dt;
                    self.v[i] += eps * fy * dt;
                    self.ws[i] += eps * fz * dt;
                }
            }
        }
    }

    /// Drive the upwind face (and the cell behind it) at the chosen wind speed,
    /// in the +X direction. Speed is expressed in cells/second.
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

    /// Viscous diffusion via Jacobi relaxation: pulls each cell toward the
    /// average of its fluid neighbors (solid neighbors are skipped so momentum
    /// isn't smeared into obstacles).
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

    /// Semi-Lagrangian backtrace advection of `field` (previous state in `q0`).
    #[allow(clippy::too_many_arguments)]
    fn advect(
        w: usize,
        h: usize,
        d: usize,
        field: &mut [f32],
        q0: &[f32],
        u: &[f32],
        v: &[f32],
        ws: &[f32],
        dt: f32,
        grid: &Grid3D,
    ) {
        if w < 3 || h < 3 || d < 3 {
            return;
        }
        let idx = |x: usize, y: usize, z: usize| z * w * h + y * w + x;
        for z in 1..d - 1 {
            for y in 1..h - 1 {
                for x in 1..w - 1 {
                    let i = idx(x, y, z);
                    if grid.solid[i] {
                        continue;
                    }
                    // Backtrace.
                    let mut px = x as f32 - dt * u[i];
                    let mut py = y as f32 - dt * v[i];
                    let mut pz = z as f32 - dt * ws[i];
                    px = px.clamp(0.5, w as f32 - 1.5);
                    py = py.clamp(0.5, h as f32 - 1.5);
                    pz = pz.clamp(0.5, d as f32 - 1.5);

                    let x0 = px.floor() as usize;
                    let y0 = py.floor() as usize;
                    let z0 = pz.floor() as usize;
                    let x1 = x0 + 1;
                    let y1 = y0 + 1;
                    let z1 = z0 + 1;

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
                    field[i] = l(c0, c1, sz);
                }
            }
        }
    }

    /// Pressure projection: subtract the gradient of pressure so the velocity
    /// field becomes (approximately) divergence-free. Solid cells act as
    /// Neumann walls — the flow is forced to go around them.
    fn project(&mut self, grid: &Grid3D) {
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
                        self.div[i] = 0.0;
                        self.p[i] = 0.0;
                        continue;
                    }
                    self.div[i] = -0.5
                        * ((self.u[idx(x + 1, y, z)] - self.u[idx(x - 1, y, z)])
                            + (self.v[idx(x, y + 1, z)] - self.v[idx(x, y - 1, z)])
                            + (self.ws[idx(x, y, z + 1)] - self.ws[idx(x, y, z - 1)]));
                    self.p[i] = 0.0;
                }
            }
        }

        for _ in 0..PRESSURE_ITERS {
            for z in 1..d - 1 {
                for y in 1..h - 1 {
                    for x in 1..w - 1 {
                        let i = idx(x, y, z);
                        if grid.solid[i] {
                            continue;
                        }
                        let me = self.p[i];
                        // Treat solid neighbors as zero normal gradient by
                        // substituting this cell's own pressure.
                        let nb = |nx: usize, ny: usize, nz: usize| {
                            let j = idx(nx, ny, nz);
                            if grid.solid[j] { me } else { self.p[j] }
                        };
                        let sum = nb(x + 1, y, z)
                            + nb(x - 1, y, z)
                            + nb(x, y + 1, z)
                            + nb(x, y - 1, z)
                            + nb(x, y, z + 1)
                            + nb(x, y, z - 1);
                        self.p[i] = (self.div[i] + sum) / 6.0;
                    }
                }
            }
        }

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
