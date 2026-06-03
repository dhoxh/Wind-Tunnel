use std::time::{Duration, Instant};

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::window::PresentMode;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

mod fluid;
mod geometry;
mod grid;
mod renderer;

use fluid::FluidSim;
use geometry::{ImportRequest, ModelPlacement, ObstacleViz, PendingLoad};
use grid::{CELL_SIZE, MM_PER_CELL};
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

/// User-facing tunables that aren't part of the solver or visualization state.
#[derive(Resource)]
pub struct Config {
    /// Obstacle ground clearance, in millimeters (relative to a nominal height).
    pub ride_height_mm: f32,
    /// Model heading about the vertical axis, in degrees. The default faces the
    /// model into the oncoming wind.
    pub model_yaw_deg: f32,
    /// Streamlines are only seeded up to this height above the ground, in mm,
    /// so the wind reads as a sensible band instead of filling the whole domain.
    pub max_wind_height_mm: f32,
    /// Frame-rate cap; `0` means uncapped. VSync is off so this governs pacing.
    pub fps_cap: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            ride_height_mm: 40.0,
            model_yaw_deg: -90.0,
            max_wind_height_mm: 500.0,
            fps_cap: 144.0,
        }
    }
}

/// Request flag: rebuild the default cube obstacle (drops any imported model).
#[derive(Resource, Default)]
struct RestoreCubeRequest(bool);

/// True while the active obstacle is the built-in cube (not an imported model).
#[derive(Resource, Default)]
struct ObstacleIsCube(bool);

/// Set each frame from egui so world interactions ignore clicks over the UI.
#[derive(Resource, Default)]
struct PointerOverUi(bool);

/// Tracks wall-clock time for the frame-rate limiter.
#[derive(Resource)]
struct FrameLimiter {
    last: Instant,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "RustFlow — Interactive Wind Tunnel".into(),
                // VSync off; pacing is handled by the FPS-cap slider.
                present_mode: PresentMode::AutoNoVsync,
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
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
        .insert_resource(FrameLimiter { last: Instant::now() })
        .init_resource::<VizSettings>()
        .init_resource::<Config>()
        .init_resource::<ImportRequest>()
        .init_resource::<RestoreCubeRequest>()
        .init_resource::<ObstacleIsCube>()
        .init_resource::<PointerOverUi>()
        .add_systems(
            Startup,
            (setup_3d, spawn_default_cube, spawn_fps_counter).chain(),
        )
        .add_systems(
            Update,
            (
                camera_look,
                camera_move,
                update_fps,
                handle_controls,
                step_fluid,
                renderer::draw_streamlines,
                geometry::handle_import_request,
                geometry::process_loading,
                geometry::apply_placement,
                manage_cube,
                toggle_obstacle_visibility,
                spin_wheels,
            ),
        )
        .add_systems(EguiPrimaryContextPass, settings_ui)
        .add_systems(Last, frame_limiter)
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

    // Ground plane at y = 0 (the tunnel floor).
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

/// Number of cells the default cube spans in each axis.
const CUBE_CELLS: usize = 6;

/// Seed the default cube into the grid and render its voxels, sitting on the
/// ground at the configured ride height.
fn build_cube(
    commands: &mut Commands,
    grid: &mut grid::Grid3D,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    ride_mm: f32,
) {
    let start_y = (ride_mm / MM_PER_CELL).round() as usize;

    let cx = grid.width / 3;
    let cz = grid.depth / 2;
    let x0 = cx.saturating_sub(CUBE_CELLS / 2);
    let z0 = cz.saturating_sub(CUBE_CELLS / 2);

    for y in start_y..(start_y + CUBE_CELLS).min(grid.height) {
        for z in z0..(z0 + CUBE_CELLS).min(grid.depth) {
            for x in x0..(x0 + CUBE_CELLS).min(grid.width) {
                let i = grid.index(x, y, z);
                grid.solid[i] = true;
            }
        }
    }

    let voxel_mesh = meshes.add(Cuboid::new(CELL_SIZE, CELL_SIZE, CELL_SIZE));
    let voxel_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.2, 0.2),
        perceptual_roughness: 0.95,
        ..default()
    });

    for z in 0..grid.depth {
        for y in 0..grid.height {
            for x in 0..grid.width {
                if grid.solid[grid.index(x, y, z)] {
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

fn spawn_default_cube(
    mut commands: Commands,
    mut grid: ResMut<grid::Grid3D>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<Config>,
    mut is_cube: ResMut<ObstacleIsCube>,
) {
    build_cube(
        &mut commands,
        &mut grid,
        &mut meshes,
        &mut materials,
        config.ride_height_mm,
    );
    is_cube.0 = true;
}

/// Rebuild the cube on explicit request, or when ride height changes while the
/// cube (not a model) is the active obstacle.
fn manage_cube(
    mut commands: Commands,
    mut grid: ResMut<grid::Grid3D>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<Config>,
    mut restore: ResMut<RestoreCubeRequest>,
    mut is_cube: ResMut<ObstacleIsCube>,
    model: Option<Res<ModelPlacement>>,
    pending: Option<Res<PendingLoad>>,
    obstacles: Query<Entity, With<ObstacleViz>>,
    mut last_ride: Local<f32>,
) {
    let restore_now = restore.0;
    restore.0 = false;
    let model_active = model.is_some() || pending.is_some();

    if restore_now {
        commands.remove_resource::<ModelPlacement>();
        commands.remove_resource::<PendingLoad>();
    } else if model_active {
        is_cube.0 = false;
        return;
    }

    let ride_changed = (config.ride_height_mm - *last_ride).abs() > 1e-3;
    let build = restore_now || (is_cube.0 && ride_changed);
    if !build {
        return;
    }

    for e in &obstacles {
        commands.entity(e).despawn();
    }
    grid.clear_solids();
    build_cube(
        &mut commands,
        &mut grid,
        &mut meshes,
        &mut materials,
        config.ride_height_mm,
    );
    is_cube.0 = true;
    *last_ride = config.ride_height_mm;
}

/// Advance the Navier–Stokes solver each frame.
fn step_fluid(mut sim: ResMut<FluidSim>, grid: Res<grid::Grid3D>, time: Res<Time>) {
    let dt = time.delta_secs();
    sim.step(&grid, dt);
}

/// Keyboard shortcuts that mirror the settings menu.
fn handle_controls(
    keys: Res<ButtonInput<KeyCode>>,
    mut sim: ResMut<FluidSim>,
    mut viz: ResMut<VizSettings>,
    mut import: ResMut<ImportRequest>,
    mut restore: ResMut<RestoreCubeRequest>,
) {
    if keys.just_pressed(KeyCode::Enter) {
        sim.running = !sim.running;
    }
    if keys.just_pressed(KeyCode::KeyR) {
        sim.reset();
    }
    if keys.just_pressed(KeyCode::KeyP) {
        viz.show_streamlines = !viz.show_streamlines;
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
        restore.0 = true;
    }
}

/// The egui settings panel.
fn settings_ui(
    mut contexts: EguiContexts,
    mut sim: ResMut<FluidSim>,
    mut viz: ResMut<VizSettings>,
    mut config: ResMut<Config>,
    mut import: ResMut<ImportRequest>,
    mut restore: ResMut<RestoreCubeRequest>,
    mut pointer: ResMut<PointerOverUi>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::Window::new("Wind Tunnel")
        .default_pos(egui::pos2(12.0, 12.0))
        .resizable(false)
        .show(ctx, |ui| {
            ui.heading("Airflow");
            // 0–230 mph expressed in m/s (230 mph ≈ 102.8 m/s).
            ui.add(egui::Slider::new(&mut sim.wind_speed, 0.0..=103.0).text("Speed (m/s)"));
            ui.label(format!("≈ {:.0} mph", sim.wind_speed * 2.2369));
            ui.add(egui::Slider::new(&mut sim.viscosity, 0.0..=5.0).text("Viscosity"));
            ui.add(egui::Slider::new(&mut sim.turbulence, 0.0..=3.0).text("Turbulence"));
            ui.horizontal(|ui| {
                ui.checkbox(&mut sim.running, "Run");
                if ui.button("Reset flow").clicked() {
                    sim.reset();
                }
            });

            ui.separator();
            ui.heading("Model");
            ui.add(
                egui::Slider::new(&mut config.ride_height_mm, 20.0..=150.0)
                    .text("Ride height (mm)"),
            );
            ui.add(
                egui::Slider::new(&mut config.model_yaw_deg, -180.0..=180.0).text("Heading (deg)"),
            );
            ui.horizontal(|ui| {
                if ui.button("Import GLB / glTF…").clicked() {
                    import.0 = true;
                }
                if ui.button("Restore cube").clicked() {
                    restore.0 = true;
                }
            });

            ui.separator();
            ui.heading("Visualization");
            ui.checkbox(&mut viz.show_streamlines, "Streamlines");
            ui.checkbox(&mut viz.color_by_speed, "Color by speed (red = stagnation)");
            ui.checkbox(&mut viz.show_obstacle, "Show obstacle");
            ui.checkbox(&mut viz.spin_wheels, "Spin wheels");
            ui.add(egui::Slider::new(&mut viz.density, 4..=36).text("Streamline density"));
            ui.add(
                egui::Slider::new(&mut config.max_wind_height_mm, 100.0..=2400.0)
                    .text("Max wind height (mm)"),
            );

            ui.separator();
            ui.heading("Performance");
            ui.add(egui::Slider::new(&mut config.fps_cap, 30.0..=300.0).text("FPS cap"));
        });

    pointer.0 = ctx.wants_pointer_input();
}

/// Spin imported wheels about the car's lateral axis at a rate proportional to
/// wind speed. Wheels are found heuristically by node name (glTF `Name`).
fn spin_wheels(
    time: Res<Time>,
    sim: Res<FluidSim>,
    viz: Res<VizSettings>,
    model: Option<Res<ModelPlacement>>,
    mut wheels: Query<(&Name, &mut Transform, &GlobalTransform)>,
) {
    if !viz.spin_wheels || model.is_none() || !sim.running {
        return;
    }

    // Visual spin rate (rad/s); scales with wind speed.
    let dtheta = sim.wind_speed * 0.15 * time.delta_secs();
    if dtheta.abs() < 1e-6 {
        return;
    }
    // Wheels roll about the lateral (Z) axis — the flow runs along +X.
    let dq_world = Quat::from_axis_angle(Vec3::Z, dtheta);

    for (name, mut transform, global) in &mut wheels {
        let n = name.as_str().to_ascii_lowercase();
        if !(n.contains("wheel") || n.contains("tyre") || n.contains("tire") || n.contains("rim")) {
            continue;
        }
        // Convert the world-space spin into this node's local frame so it spins
        // correctly regardless of where it sits in the model hierarchy.
        let parent_rot = global.rotation() * transform.rotation.inverse();
        let delta_local = parent_rot.inverse() * dq_world * parent_rot;
        transform.rotation = delta_local * transform.rotation;
    }
}

/// Apply the obstacle show/hide toggle.
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

/// Limit the frame rate to `Config::fps_cap` (VSync is off).
fn frame_limiter(mut limiter: ResMut<FrameLimiter>, config: Res<Config>) {
    if config.fps_cap > 0.0 {
        let target = Duration::from_secs_f32(1.0 / config.fps_cap);
        let elapsed = limiter.last.elapsed();
        if elapsed < target {
            std::thread::sleep(target - elapsed);
        }
    }
    limiter.last = Instant::now();
}

fn camera_look(
    mut mouse_motion_events: MessageReader<MouseMotion>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    pointer: Res<PointerOverUi>,
    mut query: Query<(&mut Transform, &mut FlyCamera)>,
) {
    // Don't rotate the camera when the user is dragging in the settings panel.
    if pointer.0 || !mouse_buttons.pressed(MouseButton::Left) {
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
