use bevy::prelude::*;

pub const GRID_WIDTH: usize = 12;
pub const GRID_HEIGHT: usize = 8;
pub const GRID_DEPTH: usize = 8;
pub const CELL_SIZE: f32 = 0.8;

#[derive(Resource)]
pub struct Grid3D {
    pub width: usize,
    pub height: usize,
    pub depth: usize,

    pub density: Vec<f32>,
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

    pub fn index(&self, x: usize, y: usize, z: usize) -> usize {
        z * self.width * self.height + y * self.width + x
    }
}
