//! Wind visualization as continuous **streamlines**, the way a real wind tunnel
//! reveals flow with smoke rakes: a regular lattice of seed points on the
//! upwind face, each traced through the velocity field into a smooth line that
//! bends over and around the obstacle and trails into the wake. Lines are drawn
//! with Bevy gizmos and colored by local speed (blue = slow -> red = fast).

use bevy::prelude::*;

use crate::fluid::FluidSim;
use crate::grid::{Grid3D, CELL_SIZE, MM_PER_CELL};
use crate::Config;

/// How far (in cells) each streamline integration step advances.
const STEP_CELLS: f32 = 0.4;
/// Maximum samples per streamline before we stop tracing.
const MAX_STEPS: usize = 220;

/// Display options, driven from the settings menu.
#[derive(Resource)]
pub struct VizSettings {
    pub show_streamlines: bool,
    pub show_obstacle: bool,
    pub color_by_speed: bool,
    /// Seed lines per axis on the inlet face (density x density lines).
    pub density: u32,
}

impl Default for VizSettings {
    fn default() -> Self {
        Self {
            show_streamlines: true,
            show_obstacle: true,
            color_by_speed: true,
            density: 12,
        }
    }
}

/// Trace and draw the streamlines for the current velocity field each frame.
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

    let origin = grid.world_origin();
    let max_speed = (sim.wind_speed / CELL_SIZE).max(0.001);
    let n = settings.density.max(2);

    // Cap the seeded band height (in cells) so the wind reads as a sensible
    // layer near the ground rather than filling the whole tall domain.
    let top_y = (config.max_wind_height_mm / MM_PER_CELL).clamp(1.0, sim.h as f32 - 2.0);

    // Distribute seeds across the inlet cross-section (Y x Z), just inside the
    // walls so lines don't start clipped into the boundary.
    for iy in 0..n {
        for iz in 0..n {
            let fy = (iy as f32 + 0.5) / n as f32;
            let fz = (iz as f32 + 0.5) / n as f32;
            let y = 1.0 + fy * (top_y - 1.0).max(0.0);
            let z = 1.0 + fz * (sim.d as f32 - 3.0);

            let mut pos = Vec3::new(0.6, y, z);
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
                let color = if settings.color_by_speed {
                    speed_color(t)
                } else {
                    Color::srgb(0.6, 0.8, 1.0)
                };
                pts.push((world, color));

                // Advance by a fixed arc length for evenly spaced, smooth lines.
                pos += (vel / speed) * STEP_CELLS;

                if pos.x >= sim.w as f32 - 1.0
                    || pos.y <= 0.5
                    || pos.y >= sim.h as f32 - 1.0
                    || pos.z <= 0.5
                    || pos.z >= sim.d as f32 - 1.0
                {
                    let world = origin + pos * CELL_SIZE;
                    pts.push((world, Color::srgb(0.6, 0.8, 1.0)));
                    break;
                }
            }

            if pts.len() >= 2 {
                gizmos.linestrip_gradient(pts);
            }
        }
    }
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
