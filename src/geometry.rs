//! Import of user-supplied 3D models (GLB / glTF — e.g. downloaded from
//! Sketchfab) via a native file-picker, and voxelization of the imported mesh
//! into the solver's solid mask so the wind actually flows around it.
//!
//! Pipeline when the user requests an import:
//!   1. open a native file dialog (`rfd`) and let them choose a `.glb`/`.gltf`,
//!   2. copy the file into the asset folder and load it as a Bevy scene,
//!   3. auto-fit the model into the middle of the tunnel,
//!   4. surface-voxelize every triangle into `Grid3D::solid`.

use std::path::PathBuf;

use bevy::prelude::*;
use bevy::mesh::{Indices, VertexAttributeValues};
use bevy::scene::SceneRoot;

use crate::grid::{Grid3D, CELL_SIZE};

/// Marker for anything that counts as "the obstacle" for the show/hide toggle:
/// the default cube voxels and imported model roots both carry it.
#[derive(Component)]
pub struct ObstacleViz;

/// Set to `true` (by a keypress in `main`) to request opening the file dialog.
#[derive(Resource, Default)]
pub struct ImportRequest(pub bool);

#[derive(PartialEq, Eq, Clone, Copy)]
enum ImportStage {
    /// Scene spawned, waiting for its meshes to finish loading.
    WaitingForMeshes,
    /// Model has been auto-fit; waiting one frame for transform propagation.
    Fitted,
}

/// Tracks the model currently being imported until it is fully voxelized.
#[derive(Resource)]
pub struct PendingImport {
    root: Entity,
    stage: ImportStage,
}

/// Opens the file dialog when requested and kicks off an async scene load.
pub fn handle_import_request(
    mut request: ResMut<ImportRequest>,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut grid: ResMut<Grid3D>,
    obstacles: Query<Entity, With<ObstacleViz>>,
    pending: Option<Res<PendingImport>>,
) {
    if !request.0 {
        return;
    }
    request.0 = false;

    // Don't start a second import while one is still being processed.
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

    commands.insert_resource(PendingImport {
        root,
        stage: ImportStage::WaitingForMeshes,
    });
}

/// Drives auto-fit and voxelization across the frames after a scene is spawned.
pub fn process_pending_import(
    mut commands: Commands,
    pending: Option<ResMut<PendingImport>>,
    mut grid: ResMut<Grid3D>,
    meshes: Res<Assets<Mesh>>,
    children_q: Query<&Children>,
    mesh_q: Query<(&Mesh3d, &GlobalTransform)>,
    mut transforms: Query<&mut Transform>,
) {
    let Some(mut pending) = pending else {
        return;
    };

    // Gather all (mesh, world-transform) pairs under the model root.
    let mut entries: Vec<(&Mesh, GlobalTransform)> = Vec::new();
    let mut stack = vec![pending.root];
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

    if entries.is_empty() {
        // Meshes not loaded yet — try again next frame.
        return;
    }

    // Combined world-space AABB of all the model's vertices.
    let (mut min, mut max) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for (mesh, gt) in &entries {
        if let Some(positions) = mesh_positions(mesh) {
            for p in positions {
                let w = gt.transform_point(Vec3::from(*p));
                min = min.min(w);
                max = max.max(w);
            }
        }
    }
    if !min.is_finite() || !max.is_finite() || (max - min).length() < 1e-4 {
        return;
    }

    match pending.stage {
        ImportStage::WaitingForMeshes => {
            // Auto-fit: scale + center the model into the tunnel.
            let size = max - min;
            let center = (min + max) * 0.5;

            let target_x = grid.width as f32 * CELL_SIZE * 0.40;
            let target_y = grid.height as f32 * CELL_SIZE * 0.55;
            let target_z = grid.depth as f32 * CELL_SIZE * 0.55;
            let scale = (target_x / size.x.max(1e-4))
                .min(target_y / size.y.max(1e-4))
                .min(target_z / size.z.max(1e-4));

            let domain_center =
                grid.cell_to_world(grid.width / 2, grid.height / 2, grid.depth / 2);
            let translation = domain_center - scale * center;

            if let Ok(mut t) = transforms.get_mut(pending.root) {
                *t = Transform::from_translation(translation).with_scale(Vec3::splat(scale));
            }
            pending.stage = ImportStage::Fitted;
        }
        ImportStage::Fitted => {
            // Transforms have now propagated; voxelize at final placement.
            voxelize(&entries, &mut grid);
            let n: usize = grid.solid.iter().filter(|s| **s).count();
            info!("Model voxelized into {n} solid cells.");
            commands.remove_resource::<PendingImport>();
        }
    }
}

/// Surface-voxelize triangles: walk each triangle finely enough that every
/// crossed cell gets marked solid.
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

            // Subdivide proportional to the triangle's size in cells.
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

/// Copy the picked file into `assets/imported/` so the AssetServer can load it
/// with a relative path. Returns the asset-relative path.
fn stage_into_assets(src: &std::path::Path) -> std::io::Result<String> {
    let file_name = src
        .file_name()
        .map(|n| n.to_string_lossy().to_string())
        .unwrap_or_else(|| "model.glb".to_string());
    let dest_dir = std::path::Path::new("assets/imported");
    std::fs::create_dir_all(dest_dir)?;
    let dest = dest_dir.join(&file_name);
    std::fs::copy(src, &dest)?;
    Ok(format!("imported/{file_name}"))
}
