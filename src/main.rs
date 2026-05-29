use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::window::PresentMode;

mod fluid;
mod geometry;
mod grid;
mod renderer;

use fluid::FluidSim;
use geometry::{ImportRequest, ObstacleViz};
use renderer::VizSettings;

#[derive(Component)]
struct Voxel;

#[derive(Component)]
struct FlyCamera {
    yaw: f32,
    pitch: f32,
    speed: f32,
    sensitivity: f32,
}

/// Request flag: rebuild the default cube obstacle (used to clear an import).
#[derive(Resource, Default)]
struct ResetObstacleRequest(bool);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "RustFlow — Interactive Wind Tunnel".into(),
                present_mode: PresentMode::AutoVsync,
                ..default()
            }),
            ..default()
        }))
        .insert_resource(grid::Grid3D::new(
            grid::GRID_WIDTH,
            grid::GRID_HEIGHT,
            grid::GRID_DEPTH,
        ))
        .insert_resource(FluidSim::new(
            grid::GRID_WIDTH,
            grid::GRID_HEIGHT,
            grid::GRID_DEPTH,
        ))
        .init_resource::<VizSettings>()
        .init_resource::<ImportRequest>()
        .init_resource::<ResetObstacleRequest>()
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_systems(
            Startup,
            (
                setup_3d,
                seed_cube_obstacle,
                spawn_voxel_obstacle,
                renderer::setup_particles,
                spawn_fps_counter,
                spawn_hud,
            )
                .chain(),
        )
        .add_systems(
            Update,
            (
                camera_look,
                camera_move,
                update_fps,
                handle_controls,
                step_fluid,
                renderer::update_particles,
                geometry::handle_import_request,
                geometry::process_pending_import,
                handle_reset_obstacle,
                toggle_obstacle_visibility,
                update_hud,
            ),
        )
        .run();
}

fn setup_3d(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-18.0, 12.0, 22.0).looking_at(Vec3::new(0.0, 4.0, 0.0), Vec3::Y),
        FlyCamera {
            yaw: -0.6,
            pitch: -0.35,
            speed: 12.0,
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
        Mesh3d(meshes.add(Plane3d::default().mesh().size(60.0, 60.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.12, 0.12, 0.14),
            perceptual_roughness: 0.9,
            ..default()
        })),
        Transform::default(),
    ));
}

/// Seed the default block obstacle into the solid grid.
fn seed_cube_obstacle(mut grid: ResMut<grid::Grid3D>) {
    let cx = grid.width / 3;
    let cy = grid.height / 2;
    let cz = grid.depth / 2;
    let r = 3;

    for z in cz.saturating_sub(r)..=(cz + r).min(grid.depth - 1) {
        for y in cy.saturating_sub(r)..=(cy + r).min(grid.height - 1) {
            for x in cx.saturating_sub(r)..=(cx + r).min(grid.width - 1) {
                let idx = grid.index(x, y, z);
                grid.density[idx] = 1.0;
                grid.solid[idx] = true;
            }
        }
    }
}

/// Render one red cube per solid cell (only used for the default obstacle).
fn spawn_voxel_obstacle(
    mut commands: Commands,
    grid: Res<grid::Grid3D>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let voxel_mesh = meshes.add(Cuboid::new(
        grid::CELL_SIZE,
        grid::CELL_SIZE,
        grid::CELL_SIZE,
    ));

    let voxel_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.2, 0.2),
        perceptual_roughness: 0.95,
        ..default()
    });

    for z in 0..grid.depth {
        for y in 0..grid.height {
            for x in 0..grid.width {
                let i = grid.index(x, y, z);
                if grid.solid[i] {
                    commands.spawn((
                        Voxel,
                        ObstacleViz,
                        Mesh3d(voxel_mesh.clone()),
                        MeshMaterial3d(voxel_material.clone()),
                        Transform::from_translation(grid.cell_to_world(x, y, z)),
                    ));
                }
            }
        }
    }
}

/// Advance the Navier–Stokes solver each frame.
fn step_fluid(mut sim: ResMut<FluidSim>, grid: Res<grid::Grid3D>, time: Res<Time>) {
    let dt = time.delta_secs();
    sim.step(&grid, dt);
}

/// Keyboard controls for the simulation and visualization toggles.
fn handle_controls(
    keys: Res<ButtonInput<KeyCode>>,
    mut sim: ResMut<FluidSim>,
    mut viz: ResMut<VizSettings>,
    mut import: ResMut<ImportRequest>,
    mut reset_obstacle: ResMut<ResetObstacleRequest>,
) {
    if keys.just_pressed(KeyCode::Enter) {
        sim.running = !sim.running;
    }
    if keys.just_pressed(KeyCode::KeyR) {
        sim.reset();
    }
    if keys.just_pressed(KeyCode::BracketRight) {
        sim.wind_speed = (sim.wind_speed + 1.0).min(25.0);
    }
    if keys.just_pressed(KeyCode::BracketLeft) {
        sim.wind_speed = (sim.wind_speed - 1.0).max(0.0);
    }
    if keys.just_pressed(KeyCode::Period) {
        sim.viscosity = (sim.viscosity + 0.1).min(5.0);
    }
    if keys.just_pressed(KeyCode::Comma) {
        sim.viscosity = (sim.viscosity - 0.1).max(0.0);
    }
    if keys.just_pressed(KeyCode::KeyP) {
        viz.show_particles = !viz.show_particles;
    }
    if keys.just_pressed(KeyCode::KeyB) {
        viz.show_obstacle = !viz.show_obstacle;
    }
    if keys.just_pressed(KeyCode::KeyC) {
        viz.color_by_speed = !viz.color_by_speed;
    }
    if keys.just_pressed(KeyCode::KeyO) {
        import.0 = true;
    }
    if keys.just_pressed(KeyCode::KeyX) {
        reset_obstacle.0 = true;
    }
}

/// Rebuild the default cube obstacle when the user clears an imported model.
fn handle_reset_obstacle(
    mut request: ResMut<ResetObstacleRequest>,
    mut commands: Commands,
    mut grid: ResMut<grid::Grid3D>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    obstacles: Query<Entity, With<ObstacleViz>>,
) {
    if !request.0 {
        return;
    }
    request.0 = false;

    for e in &obstacles {
        commands.entity(e).despawn();
    }
    grid.clear_solids();

    // Re-seed and re-render the default block.
    let cx = grid.width / 3;
    let cy = grid.height / 2;
    let cz = grid.depth / 2;
    let r = 3;
    for z in cz.saturating_sub(r)..=(cz + r).min(grid.depth - 1) {
        for y in cy.saturating_sub(r)..=(cy + r).min(grid.height - 1) {
            for x in cx.saturating_sub(r)..=(cx + r).min(grid.width - 1) {
                let idx = grid.index(x, y, z);
                grid.solid[idx] = true;
            }
        }
    }

    let voxel_mesh = meshes.add(Cuboid::new(
        grid::CELL_SIZE,
        grid::CELL_SIZE,
        grid::CELL_SIZE,
    ));
    let voxel_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.2, 0.2),
        perceptual_roughness: 0.95,
        ..default()
    });
    for z in 0..grid.depth {
        for y in 0..grid.height {
            for x in 0..grid.width {
                let i = grid.index(x, y, z);
                if grid.solid[i] {
                    commands.spawn((
                        Voxel,
                        ObstacleViz,
                        Mesh3d(voxel_mesh.clone()),
                        MeshMaterial3d(voxel_material.clone()),
                        Transform::from_translation(grid.cell_to_world(x, y, z)),
                    ));
                }
            }
        }
    }
}

/// Apply the obstacle show/hide toggle to all obstacle entities.
fn toggle_obstacle_visibility(
    viz: Res<VizSettings>,
    mut query: Query<&mut Visibility, With<ObstacleViz>>,
) {
    let target = if viz.show_obstacle {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    for mut v in &mut query {
        if *v != target {
            *v = target;
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

#[derive(Component)]
struct FpsText;

fn spawn_fps_counter(mut commands: Commands) {
    commands.spawn((
        Text::new("FPS: "),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            right: Val::Px(10.0),
            top: Val::Px(10.0),
            ..default()
        },
        FpsText,
    ));
}

fn update_fps(diagnostics: Res<DiagnosticsStore>, mut query: Query<&mut Text, With<FpsText>>) {
    let Ok(mut text) = query.single_mut() else {
        return;
    };

    if let Some(fps) = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
    {
        *text = Text::new(format!("FPS: {:.0}", fps));
    }
}

#[derive(Component)]
struct HudText;

fn spawn_hud(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextFont {
            font_size: 15.0,
            ..default()
        },
        TextColor(Color::srgb(0.85, 0.9, 1.0)),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(10.0),
            top: Val::Px(10.0),
            ..default()
        },
        HudText,
    ));
}

fn update_hud(
    sim: Res<FluidSim>,
    viz: Res<VizSettings>,
    mut query: Query<&mut Text, With<HudText>>,
) {
    let Ok(mut text) = query.single_mut() else {
        return;
    };
    let on = |b: bool| if b { "ON" } else { "OFF" };
    *text = Text::new(format!(
        "RustFlow — Wind Tunnel\n\
         Wind speed: {:.1} m/s   ( [ / ] )      Viscosity: {:.1}   ( , / . )\n\
         Simulation: {}   (Enter)    Reset flow: R\n\
         Particles: {} (P)   Obstacle: {} (B)   Color-by-speed: {} (C)\n\
         O: import 3D model (GLB/glTF)    X: restore default block\n\
         Camera: WASD move, Space/Shift up-down, hold Left-Mouse to look",
        sim.wind_speed,
        sim.viscosity,
        if sim.running { "RUNNING" } else { "PAUSED" },
        on(viz.show_particles),
        on(viz.show_obstacle),
        on(viz.color_by_speed),
    ));
}
