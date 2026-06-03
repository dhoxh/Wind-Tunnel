//! Wind visualization as continuous **streamlines**, the way a real wind tunnel
//! reveals flow with smoke rakes: a regular lattice of seed points on the
//! upwind face, each traced through the velocity field into a smooth line that
//! bends over and around the obstacle and trails into the wake. Lines are drawn
//! with Bevy gizmos and colored by local speed (blue = slow -> red = fast).

use bevy::prelude::*;
use bevy::tasks::ComputeTaskPool;

use crate::fluid::FluidSim;
use crate::grid::{Grid3D, CELL_SIZE, MM_PER_CELL};
use crate::Config;

/// How far (in cells) each streamline integration step advances. Smaller =
/// smoother, more detailed lines that resolve tight curls and vortices.
const STEP_CELLS: f32 = 0.3;
/// Maximum samples per streamline before we stop tracing.
const MAX_STEPS: usize = 380;

/// Display options, driven from the settings menu.
#[derive(Resource)]
pub struct VizSettings {
    pub show_streamlines: bool,
    pub show_obstacle: bool,
    pub color_by_speed: bool,
    /// Spin imported wheels at a rate proportional to wind speed.
    pub spin_wheels: bool,
    /// Seed lines per axis on the inlet face (density x density lines).
    pub density: u32,
}

impl Default for VizSettings {
    fn default() -> Self {
        Self {
            show_streamlines: true,
            show_obstacle: true,
            color_by_speed: true,
            spin_wheels: true,
            density: 20,
        }
    }
}

/// Trace and draw the streamlines for the current velocity field each frame.
///
/// Seeds form a tight, dense rake sized to the obstacle's frontal area (a bit
/// upstream of it) so the lines hug and detail the whole car rather than
/// spreading across the empty tunnel; longer traces let the wake vortices and
/// downwash off the wing develop. Tracing is parallelized across the compute
/// pool (one task per seed row); only the cheap gizmo drawing is on the main
/// thread.
pub fn draw_streamlines(
    mut gizmos: Gizmos,
    sim: Res<FluidSim>,
    grid: Res<Grid3D>,
    settings: Res<VizSettings>,
    config: Res<Config>,
) {
    if !settings.show_streamlines {
        return;
    }

    let max_speed = (sim.wind_speed / CELL_SIZE).max(0.001);
    let n = settings.density.max(2);
    let color_by_speed = settings.color_by_speed;
    let origin = grid.world_origin();

    // Seed region: hug the obstacle's frontal area when one exists, else fall
    // back to a band up to the configured max wind height.
    let (seed_x, y_lo, y_hi, z_lo, z_hi) = match solid_bbox(&grid) {
        Some((mn, mx)) => {
            let seed_x = (mn.x - 6.0).max(1.0);
            // Down to the floor (underbody) and up over the wing.
            let y_lo = 0.4_f32;
            let y_hi = (mx.y + 3.0).min(sim.h as f32 - 2.0);
            let z_lo = (mn.z - 1.5).max(1.0);
            let z_hi = (mx.z + 1.5).min(sim.d as f32 - 2.0);
            (seed_x, y_lo, y_hi, z_lo, z_hi)
        }
        None => {
            let top = (config.max_wind_height_mm / MM_PER_CELL).clamp(1.0, sim.h as f32 - 2.0);
            (0.6, 0.5, top, 1.0, sim.d as f32 - 2.0)
        }
    };

    let sim = &*sim;
    let grid = &*grid;
    let pool = ComputeTaskPool::get();

    // Each task traces one row of seeds (across Z) and returns its polylines.
    let rows: Vec<Vec<Vec<(Vec3, Color)>>> = pool.scope(|scope| {
        for iy in 0..n {
            scope.spawn(async move {
                let fy = (iy as f32 + 0.5) / n as f32;
                let y = y_lo + fy * (y_hi - y_lo).max(0.0);
                let mut lines = Vec::with_capacity(n as usize);
                for iz in 0..n {
                    let fz = (iz as f32 + 0.5) / n as f32;
                    let z = z_lo + fz * (z_hi - z_lo).max(0.0);
                    let line =
                        trace_streamline(sim, grid, origin, seed_x, y, z, max_speed, color_by_speed);
                    if line.len() >= 2 {
                        lines.push(line);
                    }
                }
                lines
            });
        }
    });

    for row in rows {
        for line in row {
            gizmos.linestrip_gradient(line);
        }
    }
}

/// Bounding box (in continuous cell coordinates) of all solid cells, or `None`
/// if there is no obstacle.
fn solid_bbox(grid: &Grid3D) -> Option<(Vec3, Vec3)> {
    let mut mn = IVec3::splat(i32::MAX);
    let mut mx = IVec3::splat(i32::MIN);
    let mut any = false;
    for z in 0..grid.depth {
        for y in 0..grid.height {
            for x in 0..grid.width {
                if grid.solid[grid.index(x, y, z)] {
                    any = true;
                    let p = IVec3::new(x as i32, y as i32, z as i32);
                    mn = mn.min(p);
                    mx = mx.max(p);
                }
            }
        }
    }
    any.then(|| (mn.as_vec3(), mx.as_vec3()))
}

/// Integrate a single streamline from one inlet seed by fixed arc-length steps.
fn trace_streamline(
    sim: &FluidSim,
    grid: &Grid3D,
    origin: Vec3,
    seed_x: f32,
    y: f32,
    z: f32,
    max_speed: f32,
    color_by_speed: bool,
) -> Vec<(Vec3, Color)> {
    let mut pos = Vec3::new(seed_x, y, z);
    let mut pts: Vec<(Vec3, Color)> = Vec::with_capacity(MAX_STEPS);

    for _ in 0..MAX_STEPS {
        let vel = sim.sample_velocity(pos.x, pos.y, pos.z);
        let speed = vel.length();
        if speed < 1e-4 {
            break;
        }

        let world = origin + pos * CELL_SIZE;
        if grid.is_solid_world(world) {
            break;
        }

        let t = (speed / max_speed).clamp(0.0, 1.0);
        // Reversed: slow/stagnation (air hitting the car) = red, fast = blue.
        let color = if color_by_speed {
            speed_color(1.0 - t)
        } else {
            Color::srgb(0.6, 0.8, 1.0)
        };
        pts.push((world, color));

        pos += (vel / speed) * STEP_CELLS;

        if pos.x >= sim.w as f32 - 1.0
            || pos.y <= 0.2
            || pos.y >= sim.h as f32 - 1.0
            || pos.z <= 0.5
            || pos.z >= sim.d as f32 - 1.0
        {
            pts.push((origin + pos * CELL_SIZE, Color::srgb(0.6, 0.8, 1.0)));
            break;
        }
    }

    pts
}

/// Blue (slow) -> cyan -> green -> yellow -> red (fast).
fn speed_color(t: f32) -> Color {
    let t = t.clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.25 {
        let k = t / 0.25;
        (0.0, k, 1.0)
    } else if t < 0.5 {
        let k = (t - 0.25) / 0.25;
        (0.0, 1.0, 1.0 - k)
    } else if t < 0.75 {
        let k = (t - 0.5) / 0.25;
        (k, 1.0, 0.0)
    } else {
        let k = (t - 0.75) / 0.25;
        (1.0, 1.0 - k, 0.0)
    };
    Color::srgb(r, g, b)
}
