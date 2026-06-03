use bevy::prelude::*;

pub const GRID_WIDTH: usize = 48;
pub const GRID_HEIGHT: usize = 28;
pub const GRID_DEPTH: usize = 28;
pub const CELL_SIZE: f32 = 0.6;

/// Real-world size of one grid cell, in millimeters. This fixes the mapping
/// between physical millimeters (ride height, wind-band height) and world units
/// so all the mm-based controls share one consistent scale. At 50 mm/cell a
/// typical ride height leaves a ~1-cell channel under the car for underbody flow.
pub const MM_PER_CELL: f32 = 50.0;

/// World units per millimeter.
pub const WORLD_PER_MM: f32 = CELL_SIZE / MM_PER_CELL;

/// Static occupancy + scalar density grid for the wind tunnel domain.
///
/// The grid is the shared "world" that the fluid solver writes velocity into
/// and that imported geometry is voxelized into (`solid`). It also owns the
/// mapping between integer cell coordinates and Bevy world-space so that the
/// solver, the obstacle voxels, and the wind particles all agree on where
/// things are.
#[derive(Resource)]
pub struct Grid3D {
    pub width: usize,
    pub height: usize,
    pub depth: usize,

    /// Smoke / dye density used for the optional density visualization.
    pub density: Vec<f32>,
    /// Marks cells occupied by an obstacle (the seeded cube or an imported model).
    pub solid: Vec<bool>,
}

impl Grid3D {
    pub fn new(width: usize, height: usize, depth: usize) -> Self {
        let size = width * height * depth;

        Self {
            width,
            height,
            depth,
            density: vec![0.0; size],
            solid: vec![false; size],
        }
    }

    #[inline]
    pub fn index(&self, x: usize, y: usize, z: usize) -> usize {
        z * self.width * self.height + y * self.width + x
    }

    /// World-space position of cell (0,0,0)'s center. The bottom row sits so
    /// its lower face aligns with the ground plane at `y = 0`.
    pub fn world_origin(&self) -> Vec3 {
        Vec3::new(
            -(self.width as f32 * CELL_SIZE) / 2.0,
            CELL_SIZE * 0.5,
            -(self.depth as f32 * CELL_SIZE) / 2.0,
        )
    }

    /// Center of the given cell in world-space.
    pub fn cell_to_world(&self, x: usize, y: usize, z: usize) -> Vec3 {
        self.world_origin()
            + Vec3::new(
                x as f32 * CELL_SIZE,
                y as f32 * CELL_SIZE,
                z as f32 * CELL_SIZE,
            )
    }

    /// Continuous grid coordinates (in cell units) for a world position.
    pub fn world_to_grid(&self, p: Vec3) -> Vec3 {
        (p - self.world_origin()) / CELL_SIZE
    }

    /// True if the (rounded) world position lands inside a solid cell.
    pub fn is_solid_world(&self, p: Vec3) -> bool {
        let g = self.world_to_grid(p);
        let (x, y, z) = (g.x.round(), g.y.round(), g.z.round());
        if x < 0.0 || y < 0.0 || z < 0.0 {
            return false;
        }
        let (x, y, z) = (x as usize, y as usize, z as usize);
        if x >= self.width || y >= self.height || z >= self.depth {
            return false;
        }
        self.solid[self.index(x, y, z)]
    }

    /// Clears any imported / seeded obstacle so a new model can be loaded.
    pub fn clear_solids(&mut self) {
        for s in self.solid.iter_mut() {
            *s = false;
        }
        for d in self.density.iter_mut() {
            *d = 0.0;
        }
    }
}
