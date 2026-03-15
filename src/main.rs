use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;

mod grid;

#[derive(Component)]
struct Voxel;

#[derive(Component)]
struct FlyCamera {
    yaw: f32,
    pitch: f32,
    speed: f32,
    sensitivity: f32,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(grid::Grid3D::new(
            grid::GRID_WIDTH,
            grid::GRID_HEIGHT,
            grid::GRID_DEPTH,
        ))
        .add_systems(
            Startup,
            (setup_3d, seed_cube_obstacle, spawn_voxel_obstacle).chain(),
        )
        .add_systems(Update, (camera_look, camera_move))
        .run();
}

fn setup_3d(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-12.0, 10.0, 18.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
        FlyCamera {
            yaw: -0.6,
            pitch: -0.35,
            speed: 8.0,
            sensitivity: 0.003,
        },
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, -0.8, 0.0)),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(30.0, 30.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.15, 0.15, 0.17),
            perceptual_roughness: 0.9,
            ..default()
        })),
        Transform::default(),
    ));

    // commands.spawn((
    //    Mesh3d(meshes.add(Cuboid::new(4.0, 4.0, 4.0))),
    //    MeshMaterial3d(materials.add(StandardMaterial {
    //        base_color: Color::srgb(0.35, 0.35, 0.38),
    //        perceptual_roughness: 0.95,
    //        ..default()
    //    })),
    //    Transform::from_xyz(0.0, 2.0, 0.0),
    //));
}

fn seed_cube_obstacle(mut grid: ResMut<grid::Grid3D>) {
    let center_x = grid.width / 2;
    let center_y = grid.height / 2;
    let center_z = grid.depth / 2;

    for z in (center_z - 1)..=(center_z + 1) {
        for y in (center_y - 1)..=(center_y + 1) {
            for x in (center_x - 1)..=(center_x + 1) {
                let idx = grid.index(x, y, z);
                grid.density[idx] = 1.0;
                grid.solid[idx] = true;
            }
        }
    }
}
fn spawn_voxel_obstacle(
    mut commands: Commands,
    grid: Res<grid::Grid3D>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let voxel_mesh = meshes.add(Cuboid::new(
        grid::CELL_SIZE * 1.0,
        grid::CELL_SIZE * 1.0,
        grid::CELL_SIZE * 1.0,
    ));

    let voxel_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.2, 0.2),
        perceptual_roughness: 0.95,
        ..default()
    });

    let x_offset = -(grid.width as f32 * grid::CELL_SIZE) / 2.0;
    let y_offset = 0.5;
    let z_offset = -(grid.depth as f32 * grid::CELL_SIZE) / 2.0;

    for z in 0..grid.depth {
        for y in 0..grid.height {
            for x in 0..grid.width {
                let i = grid.index(x, y, z);

                if grid.solid[i] {
                    let world_x = x_offset + x as f32 * grid::CELL_SIZE;
                    let world_y = y_offset + y as f32 * grid::CELL_SIZE;
                    let world_z = z_offset + z as f32 * grid::CELL_SIZE;

                    println!("spawning voxel at ({x}, {y}, {z})");

                    commands.spawn((
                        Voxel,
                        Mesh3d(voxel_mesh.clone()),
                        MeshMaterial3d(voxel_material.clone()),
                        Transform::from_xyz(world_x, world_y, world_z),
                    ));
                }
            }
        }
    }
}
fn camera_look(
    mut mouse_motion_events: MessageReader<MouseMotion>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut query: Query<(&mut Transform, &mut FlyCamera)>,
) {
    if !mouse_buttons.pressed(MouseButton::Left) {
        mouse_motion_events.clear();
        return;
    }

    let mut delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        delta += event.delta;
    }

    if delta == Vec2::ZERO {
        return;
    }

    let (mut transform, mut camera) = query.single_mut().unwrap();

    camera.yaw -= delta.x * camera.sensitivity;
    camera.pitch -= delta.y * camera.sensitivity;
    camera.pitch = camera.pitch.clamp(-1.54, 1.54);

    transform.rotation = Quat::from_rotation_y(camera.yaw) * Quat::from_rotation_x(camera.pitch);
}
fn camera_move(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut query: Query<(&mut Transform, &FlyCamera)>,
) {
    let (mut transform, camera) = query.single_mut().unwrap();

    let mut movement = Vec3::ZERO;

    let forward = transform.forward();
    let right = transform.right();

    let flat_forward = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
    let flat_right = Vec3::new(right.x, 0.0, right.z).normalize_or_zero();

    if keyboard.pressed(KeyCode::ArrowUp) || keyboard.pressed(KeyCode::KeyW) {
        movement += flat_forward;
    }
    if keyboard.pressed(KeyCode::ArrowDown) || keyboard.pressed(KeyCode::KeyS) {
        movement -= flat_forward;
    }
    if keyboard.pressed(KeyCode::ArrowLeft) || keyboard.pressed(KeyCode::KeyA) {
        movement -= flat_right;
    }
    if keyboard.pressed(KeyCode::ArrowRight) || keyboard.pressed(KeyCode::KeyD) {
        movement += flat_right;
    }

    if keyboard.pressed(KeyCode::Space) {
        movement += Vec3::Y;
    }
    if keyboard.pressed(KeyCode::ShiftLeft) {
        movement -= Vec3::Y;
    }

    if movement != Vec3::ZERO {
        transform.translation += movement.normalize() * camera.speed * time.delta_secs();
    }
}
