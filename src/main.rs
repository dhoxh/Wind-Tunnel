use bevy::input::mouse::MouseWheel;
use bevy::prelude::*;

mod fluid;
mod geometry;
mod grid;
mod renderer;

#[derive(Component)]
struct MainCamera;

#[derive(Component)]
struct Cell {
    index: usize,
}
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(grid::Grid::new(grid::GRID_WIDTH, grid::GRID_HEIGHT))
        .add_systems(Startup, (setup, seed_density))
        .add_systems(Update, (camera_movement, camera_zoom, update_cell_colors))
        .run();
}

fn setup(mut commands: Commands, grid: Res<grid::Grid>) {
    commands.spawn((Camera2d, MainCamera));

    let total_width = grid.width as f32 * grid::CELL_SIZE;
    let total_height = grid.height as f32 * grid::CELL_SIZE;

    let start_x = -total_width / 2.0 + grid::CELL_SIZE / 2.0;
    let start_y = -total_height / 2.0 + grid::CELL_SIZE / 2.0;

    for row in 0..grid.height {
        for col in 0..grid.width {
            let x = start_x + col as f32 * grid::CELL_SIZE;
            let y = start_y + row as f32 * grid::CELL_SIZE;
            let index = row * grid.width + col;

            commands.spawn((
                Cell { index },
                Sprite::from_color(
                    Color::srgb(0.2, 0.4, 0.8),
                    Vec2::splat(grid::CELL_SIZE - 2.0),
                ),
                Transform::from_xyz(x, y, 0.0),
            ));
        }
    }
}

fn seed_density(mut grid: ResMut<grid::Grid>) {
    let center_row = grid.height / 2;

    for col in 0..3 {
        let index = center_row * grid.width + col;
        grid.density[index] = 1.0;
    }
}

fn camera_movement(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut query: Query<&mut Transform, With<MainCamera>>,
) {
    let Ok(mut transform) = query.single_mut() else {
        return;
    };

    let mut direction = Vec3::ZERO;
    let speed = 500.0;

    if keyboard.pressed(KeyCode::KeyW) || keyboard.pressed(KeyCode::ArrowUp) {
        direction.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyS) || keyboard.pressed(KeyCode::ArrowDown) {
        direction.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
        direction.x -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
        direction.x += 1.0;
    }

    if direction != Vec3::ZERO {
        transform.translation += direction.normalize() * speed * time.delta_secs();
    }
}

fn camera_zoom(
    mut mouse_wheel_events: MessageReader<MouseWheel>,
    mut query: Query<&mut Projection, With<MainCamera>>,
) {
    let Ok(mut projection) = query.single_mut() else {
        return;
    };

    if let Projection::Orthographic(ref mut ortho) = *projection {
        for event in mouse_wheel_events.read() {
            ortho.scale -= event.y * 0.1;
            ortho.scale = ortho.scale.clamp(0.2, 5.0);
        }
    }
}

fn update_cell_colors(grid: Res<grid::Grid>, mut query: Query<(&Cell, &mut Sprite)>) {
    for (cell, mut sprite) in &mut query {
        let density = grid.density[cell.index].clamp(0.0, 1.0);

        sprite.color = Color::srgb(
            0.1 + density * 0.9,
            0.2 + density * 0.3,
            0.8 - density * 0.6,
        );
    }
}
