//! Wind visualization: thousands of tracer particles are seeded on the upwind
//! face and advected through the solver's velocity field. They reveal
//! streamlines, wakes, and recirculation around obstacles, and are colored by
//! local speed so fast/slow regions read at a glance.

use bevy::prelude::*;

use crate::fluid::FluidSim;
use crate::grid::{Grid3D, CELL_SIZE};

const PARTICLE_COUNT: usize = 2500;
const PARTICLE_LIFETIME: f32 = 6.0;
/// Number of discrete colors in the speed gradient.
const PALETTE_BANDS: usize = 24;

/// Tag + per-particle bookkeeping for a tracer entity.
#[derive(Component)]
pub struct WindParticle {
    /// Continuous position in grid (cell) coordinates.
    pos: Vec3,
    life: f32,
    band: usize,
}

/// Display options toggled from the keyboard (see `main`).
#[derive(Resource)]
pub struct VizSettings {
    pub show_particles: bool,
    pub show_obstacle: bool,
    pub color_by_speed: bool,
}

impl Default for VizSettings {
    fn default() -> Self {
        Self {
            show_particles: true,
            show_obstacle: true,
            color_by_speed: true,
        }
    }
}

/// Preallocated emissive materials forming a blue→red speed gradient, plus a
/// neutral material used when color-by-speed is disabled.
#[derive(Resource)]
pub struct SpeedPalette {
    bands: Vec<Handle<StandardMaterial>>,
    neutral: Handle<StandardMaterial>,
}

/// Tiny deterministic RNG so we don't pull in an external crate just to scatter
/// particles across the inlet.
#[derive(Resource)]
pub struct Rng(u64);

impl Rng {
    fn next_f32(&mut self) -> f32 {
        // xorshift64*
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        ((x.wrapping_mul(0x2545F4914F6CDD1D) >> 33) as f32) / (1u64 << 31) as f32
    }
}

/// Build the color palette and spawn the particle entities at the inlet.
pub fn setup_particles(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    grid: Res<Grid3D>,
) {
    let mut bands = Vec::with_capacity(PALETTE_BANDS);
    for i in 0..PALETTE_BANDS {
        let t = i as f32 / (PALETTE_BANDS - 1) as f32;
        let color = speed_color(t);
        bands.push(materials.add(StandardMaterial {
            base_color: color,
            emissive: LinearRgba::from(color) * 4.0,
            unlit: true,
            ..default()
        }));
    }
    let neutral = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.85, 1.0),
        emissive: LinearRgba::new(0.4, 0.6, 1.0, 1.0) * 3.0,
        unlit: true,
        ..default()
    });

    let particle_mesh = meshes.add(Sphere::new(CELL_SIZE * 0.12).mesh().ico(1).unwrap());

    let mut rng = Rng(0x9E3779B97F4A7C15);

    for _ in 0..PARTICLE_COUNT {
        let pos = random_inlet_pos(&mut rng, &grid);
        let world = grid.world_origin() + pos * CELL_SIZE;
        commands.spawn((
            WindParticle {
                pos,
                life: rng.next_f32() * PARTICLE_LIFETIME,
                band: 0,
            },
            Mesh3d(particle_mesh.clone()),
            MeshMaterial3d(bands[0].clone()),
            Transform::from_translation(world),
        ));
    }

    commands.insert_resource(SpeedPalette { bands, neutral });
    commands.insert_resource(rng);
}

/// Advect every particle through the velocity field and refresh its transform
/// and color band.
pub fn update_particles(
    time: Res<Time>,
    sim: Res<FluidSim>,
    grid: Res<Grid3D>,
    settings: Res<VizSettings>,
    palette: Res<SpeedPalette>,
    mut rng: ResMut<Rng>,
    mut query: Query<(
        &mut WindParticle,
        &mut Transform,
        &mut MeshMaterial3d<StandardMaterial>,
        &mut Visibility,
    )>,
) {
    let dt = time.delta_secs();
    let origin = grid.world_origin();
    let max_speed = (sim.wind_speed / CELL_SIZE).max(0.001);

    for (mut p, mut transform, mut mat, mut vis) in &mut query {
        *vis = if settings.show_particles {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };

        let vel = sim.sample_velocity(p.pos.x, p.pos.y, p.pos.z);
        p.pos += vel * dt;
        p.life -= dt;

        let out_of_domain = p.pos.x >= sim.w as f32 - 1.0
            || p.pos.y <= 0.0
            || p.pos.y >= sim.h as f32 - 1.0
            || p.pos.z <= 0.0
            || p.pos.z >= sim.d as f32 - 1.0;
        let world = origin + p.pos * CELL_SIZE;

        if p.life <= 0.0 || out_of_domain || grid.is_solid_world(world) {
            p.pos = random_inlet_pos(&mut rng, &grid);
            p.life = PARTICLE_LIFETIME;
        }

        transform.translation = origin + p.pos * CELL_SIZE;

        // Color by normalized speed.
        let target_band = if settings.color_by_speed {
            let speed = vel.length() / max_speed;
            ((speed * (PALETTE_BANDS - 1) as f32).round() as usize).min(PALETTE_BANDS - 1)
        } else {
            usize::MAX
        };

        if settings.color_by_speed {
            if target_band != p.band {
                p.band = target_band;
                mat.0 = palette.bands[target_band].clone();
            }
        } else if p.band != usize::MAX {
            p.band = usize::MAX;
            mat.0 = palette.neutral.clone();
        }
    }
}

/// Pick a random cell on the inlet face (low X), avoiding solid cells.
fn random_inlet_pos(rng: &mut Rng, grid: &Grid3D) -> Vec3 {
    for _ in 0..8 {
        let y = rng.next_f32() * (grid.height as f32 - 2.0) + 1.0;
        let z = rng.next_f32() * (grid.depth as f32 - 2.0) + 1.0;
        let pos = Vec3::new(1.0, y, z);
        let world = grid.world_origin() + pos * CELL_SIZE;
        if !grid.is_solid_world(world) {
            return pos;
        }
    }
    Vec3::new(1.0, grid.height as f32 * 0.5, grid.depth as f32 * 0.5)
}

/// Blue (slow) → cyan → green → yellow → red (fast).
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
