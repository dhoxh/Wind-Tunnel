use bevy::prelude::*;

pub const GRID_WIDTH: usize = 48;
pub const GRID_HEIGHT: usize = 32;
pub const GRID_DEPTH: usize = 32;
/// Default (initial) cell size in world units, before the grid is fitted to an
/// object. Once an obstacle exists the grid shrinks to wrap it, so the working
/// cell size is usually much smaller (and stored in `Grid3D::cell`).
pub const CELL_SIZE: f32 = 0.6;

/// Inflow speed in cells/second per (m/s) of wind. Kept independent of the
/// (now dynamic) cell size so the solver's CFL behavior is stable regardless of
/// how tightly the grid is wrapped around the car.
pub const INFLOW_GAIN: f32 = 1.667;

/// Nominal full height (mm) that ride height is expressed against, so "40 mm"
/// reads like real ground clearance relative to the object's own height.
pub const REF_HEIGHT_MM: f32 = 1200.0;

/// Adaptive occupancy + scalar density grid for the wind tunnel domain.
///
/// The grid's world placement (`origin`) and resolution (`cell`) are *fitted*
/// around whatever obstacle is present (see [`Grid3D::fit_to`]) so the fixed
/// cell budget concentrates detail on the car and no work is spent simulating
/// empty tunnel far away. The cell count stays fixed so the solver's buffers
/// never need reallocating.
#[derive(Resource)]
pub struct Grid3D {
    pub width: usize,
    pub height: usize,
    pub depth: usize,

    /// World size of one cell (uniform).
    pub cell: f32,
    /// World position of cell (0,0,0)'s center.
    pub origin: Vec3,

    /// Smoke / dye density used for the optional density visualization.
    pub density: Vec<f32>,
    /// Marks cells occupied by an obstacle (the seeded cube or an imported model).
    pub solid: Vec<bool>,
}

impl Grid3D {
    pub fn new(width: usize, height: usize, depth: usize) -> Self {
        let size = width * height * depth;
        let mut g = Self {
            width,
            height,
            depth,
            cell: CELL_SIZE,
            origin: Vec3::ZERO,
            density: vec![0.0; size],
            solid: vec![false; size],
        };
        g.origin = g.default_origin();
        g
    }

    fn default_origin(&self) -> Vec3 {
        Vec3::new(
            -(self.width as f32 * self.cell) / 2.0,
            self.cell * 0.5,
            -(self.depth as f32 * self.cell) / 2.0,
        )
    }

    #[inline]
    pub fn index(&self, x: usize, y: usize, z: usize) -> usize {
        z * self.width * self.height + y * self.width + x
    }

    /// World-space position of cell (0,0,0)'s center.
    pub fn world_origin(&self) -> Vec3 {
        self.origin
    }

    /// Center of the given cell in world-space.
    pub fn cell_to_world(&self, x: usize, y: usize, z: usize) -> Vec3 {
        self.origin + Vec3::new(x as f32, y as f32, z as f32) * self.cell
    }

    /// World-space size of the whole domain.
    pub fn world_size(&self) -> Vec3 {
        Vec3::new(self.width as f32, self.height as f32, self.depth as f32) * self.cell
    }

    /// Continuous grid coordinates (in cell units) for a world position.
    pub fn world_to_grid(&self, p: Vec3) -> Vec3 {
        (p - self.origin) / self.cell
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

    /// Any obstacle present?
    pub fn has_obstacle(&self) -> bool {
        self.solid.iter().any(|s| *s)
    }

    /// Fit the grid tightly around an obstacle's world-space AABB, with room
    /// upstream, a wake downstream, and the floor anchored at `y = 0`.
    ///
    /// The cell *size* is tied to the object's height (≈ `TARGET_H_CELLS` cells
    /// tall) for consistent detail, and the per-axis cell *counts* follow the
    /// region's shape so a long wake gets more cells along X rather than
    /// coarsening the whole grid. Total cells are capped for performance, and
    /// the solid/density buffers are reallocated to the new size.
    pub fn fit_to(&mut self, bmin: Vec3, bmax: Vec3) {
        const TARGET_H_CELLS: f32 = 22.0;
        const MAX_CELLS: usize = 90_000;

        let size = (bmax - bmin).max(Vec3::splat(0.1));
        let up = size.x * 0.5;
        let down = size.x * 1.4;
        let side = size.z * 0.6;
        let top = size.y * 0.7;

        let r0 = Vec3::new(bmin.x - up, 0.0, bmin.z - side);
        let r1 = Vec3::new(bmax.x + down, bmax.y + top, bmax.z + side);
        let ext = (r1 - r0).max(Vec3::splat(0.2));

        let dims = |cell: f32| {
            let w = ((ext.x / cell).ceil() as usize).clamp(16, 112);
            let h = ((ext.y / cell).ceil() as usize).clamp(16, 48);
            let d = ((ext.z / cell).ceil() as usize).clamp(16, 72);
            (w, h, d)
        };

        // Resolution from the object's height, but not so fine that the wake
        // blows past the cell budget.
        let mut cell = (size.y / TARGET_H_CELLS).max(ext.max_element() / 112.0);
        let (mut w, mut h, mut d) = dims(cell);
        while w * h * d > MAX_CELLS {
            cell *= 1.08;
            let n = dims(cell);
            w = n.0;
            h = n.1;
            d = n.2;
        }

        // Uniform cell that covers the region within the chosen counts.
        let cell = (ext.x / w as f32)
            .max(ext.y / h as f32)
            .max(ext.z / d as f32);

        self.width = w;
        self.height = h;
        self.depth = d;
        self.cell = cell;

        let cov = Vec3::new(w as f32, h as f32, d as f32) * cell;
        let sx = r0.x - (cov.x - ext.x) * 0.5;
        let sz = r0.z - (cov.z - ext.z) * 0.5;
        self.origin = Vec3::new(sx + 0.5 * cell, 0.5 * cell, sz + 0.5 * cell);

        let n = w * h * d;
        self.solid = vec![false; n];
        self.density = vec![0.0; n];
    }

    /// Reset to the default placement (used when no obstacle is present).
    pub fn reset_placement(&mut self) {
        self.cell = CELL_SIZE;
        self.origin = self.default_origin();
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
