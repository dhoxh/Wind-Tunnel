//! Import of user-supplied 3D models (GLB / glTF, e.g. from Sketchfab) and
//! their placement in the tunnel:
//!   * a single `.glb` is copied as-is; a multi-file `.gltf` brings its whole
//!     folder (the `.bin` buffers and textures) along,
//!   * the model is auto-fit to the tunnel, rotated to face into the oncoming
//!     wind, and seated at a configurable ride height above the ground,
//!   * every triangle is surface-voxelized into `Grid3D::solid` so the solver
//!     sees the real silhouette.
//!
//! Ride height and heading can be changed live from the settings menu; the
//! model is re-seated and re-voxelized automatically.

use std::path::PathBuf;

use bevy::mesh::{Indices, VertexAttributeValues};
use bevy::prelude::*;
use bevy::scene::SceneRoot;

use crate::grid::{Grid3D, CELL_SIZE, REF_HEIGHT_MM};
use crate::Config;

/// Marker for anything that counts as "the obstacle" for the show/hide toggle:
/// the default cube voxels and imported model roots both carry it.
#[derive(Component)]
pub struct ObstacleViz;

/// Set to `true` (from the menu or `O`) to request opening the file dialog.
#[derive(Resource, Default)]
pub struct ImportRequest(pub bool);

/// A model scene has been spawned and we're waiting for its meshes to load.
#[derive(Resource)]
pub struct PendingLoad {
    root: Entity,
}

/// A loaded model, with everything needed to (re)seat and (re)voxelize it from
/// the current ride-height / heading settings without reloading the asset.
#[derive(Resource)]
pub struct ModelPlacement {
    root: Entity,
    /// Axis-aligned bounds of the raw model (identity transform), in world units.
    bbox_min: Vec3,
    bbox_max: Vec3,
    /// Uniform fit scale, fixed at import time.
    scale: f32,
    /// Settings the current transform was built from, for change detection.
    applied_ride: f32,
    applied_yaw: f32,
    /// Frames to wait before voxelizing (lets the new transform propagate).
    voxelize_in: Option<u8>,
}

/// Opens the file dialog when requested and kicks off an async scene load.
pub fn handle_import_request(
    mut request: ResMut<ImportRequest>,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut grid: ResMut<Grid3D>,
    obstacles: Query<Entity, With<ObstacleViz>>,
    pending: Option<Res<PendingLoad>>,
) {
    if !request.0 {
        return;
    }
    request.0 = false;

    if pending.is_some() {
        warn!("Import already in progress; ignoring request.");
        return;
    }

    let Some(path) = pick_model_file() else {
        info!("Model import cancelled.");
        return;
    };

    let asset_path = match stage_into_assets(&path) {
        Ok(p) => p,
        Err(e) => {
            error!("Could not stage model into assets folder: {e}");
            return;
        }
    };

    // Clear the existing obstacle (default cube or previous model).
    for e in &obstacles {
        commands.entity(e).despawn();
    }
    grid.clear_solids();
    commands.remove_resource::<ModelPlacement>();

    info!("Loading model: {asset_path}");
    let scene = asset_server.load(GltfAssetLabel::Scene(0).from_asset(asset_path));
    let root = commands
        .spawn((
            ObstacleViz,
            SceneRoot(scene),
            Transform::default(),
            Visibility::default(),
        ))
        .id();

    commands.insert_resource(PendingLoad { root });
}

/// Once the spawned scene's meshes are available, compute its bounds and fit
/// scale and promote it to a [`ModelPlacement`].
pub fn process_loading(
    mut commands: Commands,
    pending: Option<Res<PendingLoad>>,
    grid: Res<Grid3D>,
    config: Res<Config>,
    meshes: Res<Assets<Mesh>>,
    children_q: Query<&Children>,
    mesh_q: Query<(&Mesh3d, &GlobalTransform)>,
) {
    let Some(pending) = pending else {
        return;
    };

    let entries = collect_meshes(pending.root, &children_q, &mesh_q, &meshes);
    if entries.is_empty() {
        return; // not loaded yet
    }

    let Some((min, max)) = world_bbox(&entries) else {
        return;
    };

    // Fit using the bounds *after* the default heading rotation, since rotating
    // a car to face the wind swaps its long axis into the flow direction.
    let rot = Quat::from_rotation_y(config.model_yaw_deg.to_radians());
    let rsize = rotated_extents(min, max, rot);
    let target_x = grid.width as f32 * CELL_SIZE * 0.50;
    let target_y = grid.height as f32 * CELL_SIZE * 0.70;
    let target_z = grid.depth as f32 * CELL_SIZE * 0.60;
    let scale = (target_x / rsize.x.max(1e-4))
        .min(target_y / rsize.y.max(1e-4))
        .min(target_z / rsize.z.max(1e-4));

    commands.insert_resource(ModelPlacement {
        root: pending.root,
        bbox_min: min,
        bbox_max: max,
        scale,
        applied_ride: f32::MIN, // force first apply
        applied_yaw: f32::MIN,
        voxelize_in: None,
    });
    commands.remove_resource::<PendingLoad>();
}

/// Live placement: (re)seat the model when ride height / heading change, then
/// re-voxelize once the transform has propagated.
pub fn apply_placement(
    placement: Option<ResMut<ModelPlacement>>,
    config: Res<Config>,
    mut grid: ResMut<Grid3D>,
    meshes: Res<Assets<Mesh>>,
    children_q: Query<&Children>,
    mesh_q: Query<(&Mesh3d, &GlobalTransform)>,
    mut transforms: Query<&mut Transform>,
) {
    let Some(mut placement) = placement else {
        return;
    };

    // Re-seat when the relevant settings changed (or on first apply).
    if config.ride_height_mm != placement.applied_ride
        || config.model_yaw_deg != placement.applied_yaw
    {
        let s = placement.scale;
        let rot = Quat::from_rotation_y(config.model_yaw_deg.to_radians());

        let height_world = (placement.bbox_max.y - placement.bbox_min.y) * s;
        let gap = (config.ride_height_mm / REF_HEIGHT_MM) * height_world;

        let centroid = (placement.bbox_min + placement.bbox_max) * 0.5;
        let rc = rot * centroid;
        let domain_center =
            grid.cell_to_world(grid.width / 2, grid.height / 2, grid.depth / 2);

        // world = T + rot*(s*local); rotation about Y leaves Y unchanged.
        let ty = gap - s * placement.bbox_min.y; // bottom sits `gap` above ground
        let tx = domain_center.x - s * rc.x;
        let tz = domain_center.z - s * rc.z;

        if let Ok(mut t) = transforms.get_mut(placement.root) {
            *t = Transform::from_translation(Vec3::new(tx, ty, tz))
                .with_rotation(rot)
                .with_scale(Vec3::splat(s));
        }

        placement.applied_ride = config.ride_height_mm;
        placement.applied_yaw = config.model_yaw_deg;
        placement.voxelize_in = Some(1); // wait a frame for propagation
        return;
    }

    // Re-voxelize once the transform has settled.
    match placement.voxelize_in {
        Some(n) if n > 0 => placement.voxelize_in = Some(n - 1),
        Some(_) => {
            let entries = collect_meshes(placement.root, &children_q, &mesh_q, &meshes);
            grid.clear_solids();
            voxelize(&entries, &mut grid);
            let n: usize = grid.solid.iter().filter(|s| **s).count();
            info!("Model voxelized into {n} solid cells.");
            placement.voxelize_in = None;
        }
        None => {}
    }
}

/// Gather all (mesh, world-transform) pairs under an entity subtree.
fn collect_meshes<'a>(
    root: Entity,
    children_q: &Query<&Children>,
    mesh_q: &Query<(&Mesh3d, &GlobalTransform)>,
    meshes: &'a Assets<Mesh>,
) -> Vec<(&'a Mesh, GlobalTransform)> {
    let mut entries = Vec::new();
    let mut stack = vec![root];
    while let Some(e) = stack.pop() {
        if let Ok(children) = children_q.get(e) {
            stack.extend(children.iter());
        }
        if let Ok((mesh3d, gt)) = mesh_q.get(e) {
            if let Some(mesh) = meshes.get(&mesh3d.0) {
                entries.push((mesh, *gt));
            }
        }
    }
    entries
}

/// Combined world-space AABB of all the model's vertices (identity transform).
fn world_bbox(entries: &[(&Mesh, GlobalTransform)]) -> Option<(Vec3, Vec3)> {
    let (mut min, mut max) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for (mesh, gt) in entries {
        if let Some(positions) = mesh_positions(mesh) {
            for p in positions {
                let w = gt.transform_point(Vec3::from(*p));
                min = min.min(w);
                max = max.max(w);
            }
        }
    }
    if min.is_finite() && max.is_finite() && (max - min).length() > 1e-4 {
        Some((min, max))
    } else {
        None
    }
}

/// Extents of an AABB after a rotation about its centroid.
fn rotated_extents(min: Vec3, max: Vec3, rot: Quat) -> Vec3 {
    let c = (min + max) * 0.5;
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for i in 0..8 {
        let corner = Vec3::new(
            if i & 1 == 0 { min.x } else { max.x },
            if i & 2 == 0 { min.y } else { max.y },
            if i & 4 == 0 { min.z } else { max.z },
        );
        let r = rot * (corner - c) + c;
        lo = lo.min(r);
        hi = hi.max(r);
    }
    hi - lo
}

/// Surface-voxelize triangles into the solid mask.
fn voxelize(entries: &[(&Mesh, GlobalTransform)], grid: &mut Grid3D) {
    let mark = |w: Vec3, grid: &mut Grid3D| {
        let g = grid.world_to_grid(w);
        let (x, y, z) = (g.x.round(), g.y.round(), g.z.round());
        if x < 0.0 || y < 0.0 || z < 0.0 {
            return;
        }
        let (x, y, z) = (x as usize, y as usize, z as usize);
        if x < grid.width && y < grid.height && z < grid.depth {
            let i = grid.index(x, y, z);
            grid.solid[i] = true;
        }
    };

    for (mesh, gt) in entries {
        let Some(positions) = mesh_positions(mesh) else {
            continue;
        };
        let tri_indices: Vec<usize> = match mesh.indices() {
            Some(Indices::U16(v)) => v.iter().map(|&i| i as usize).collect(),
            Some(Indices::U32(v)) => v.iter().map(|&i| i as usize).collect(),
            None => (0..positions.len()).collect(),
        };

        for tri in tri_indices.chunks_exact(3) {
            let a = gt.transform_point(Vec3::from(positions[tri[0]]));
            let b = gt.transform_point(Vec3::from(positions[tri[1]]));
            let c = gt.transform_point(Vec3::from(positions[tri[2]]));

            let edge = (b - a).length().max((c - a).length()).max((c - b).length());
            let steps = ((edge / CELL_SIZE) * 2.0).ceil().max(1.0) as usize;
            for i in 0..=steps {
                for j in 0..=(steps - i) {
                    let u = i as f32 / steps as f32;
                    let v = j as f32 / steps as f32;
                    let p = a + (b - a) * u + (c - a) * v;
                    mark(p, grid);
                }
            }
        }
    }
}

fn mesh_positions(mesh: &Mesh) -> Option<&[[f32; 3]]> {
    match mesh.attribute(Mesh::ATTRIBUTE_POSITION)? {
        VertexAttributeValues::Float32x3(v) => Some(v.as_slice()),
        _ => None,
    }
}

/// Native file dialog restricted to glTF model files.
fn pick_model_file() -> Option<PathBuf> {
    rfd::FileDialog::new()
        .add_filter("glTF model", &["glb", "gltf"])
        .set_title("Select a 3D model to drop into the wind tunnel")
        .pick_file()
}

/// Stage the picked model into `assets/imported/`. A `.glb` is self-contained;
/// a `.gltf` references sibling files, so we copy its whole folder.
fn stage_into_assets(src: &std::path::Path) -> std::io::Result<String> {
    let file_name = src
        .file_name()
        .map(|n| n.to_string_lossy().to_string())
        .unwrap_or_else(|| "model.glb".to_string());
    let dest_root = std::path::Path::new("assets/imported");

    let is_gltf = src
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.eq_ignore_ascii_case("gltf"))
        .unwrap_or(false);

    if is_gltf {
        let parent = src.parent().unwrap_or(std::path::Path::new("."));
        let dest_dir = dest_root.join("model");
        if dest_dir.exists() {
            std::fs::remove_dir_all(&dest_dir)?;
        }
        copy_dir_all(parent, &dest_dir)?;
        Ok(format!("imported/model/{file_name}"))
    } else {
        std::fs::create_dir_all(dest_root)?;
        let dest = dest_root.join(&file_name);
        std::fs::copy(src, &dest)?;
        Ok(format!("imported/{file_name}"))
    }
}

/// Recursively copy a directory tree.
fn copy_dir_all(src: &std::path::Path, dest: &std::path::Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dest)?;
    for entry in std::fs::read_dir(src)? {
        let entry = entry?;
        let path = entry.path();
        let target = dest.join(entry.file_name());
        if path.is_dir() {
            copy_dir_all(&path, &target)?;
        } else {
            std::fs::copy(&path, &target)?;
        }
    }
    Ok(())
}
