use bevy::prelude::*;

pub const GRID_WIDTH: usize = 10;
pub const GRID_HEIGHT: usize = 10;
pub const CELL_SIZE: f32 = 40.0;

#[derive(Resource)]
pub struct Grid {
    pub width: usize,
    pub height: usize,

    pub velocity_x: Vec<f32>,
    pub velocity_y: Vec<f32>,
    pub pressure: Vec<f32>,
    pub density: Vec<f32>,
}

impl Grid {
    pub fn new(width: usize, height: usize) -> Self {
        let size = width * height;

        Self {
            width,
            height,
            velocity_x: vec![0.0; size],
            velocity_y: vec![0.0; size],
            pressure: vec![0.0; size],
            density: vec![0.0; size],
        }
    }
}
