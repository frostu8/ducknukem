//! Temporarily lifted from
//! https://github.com/frostu8/soochcart/blob/main/src/map.rs.

use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};

use bevy_rapier3d::prelude::*;

use super::GameState;

/// Track loading plugin.
pub struct MapPlugin;

impl Plugin for MapPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LoadMap>()
            .add_systems(
                Update,
                wait_for_assets.run_if(in_state(GameState::LoadingMap)),
            )
            .add_systems(Update, add_generate_tri_meshes)
            .add_systems(Update, generate_tri_meshes);
    }
}

/// Loads a track.
#[derive(Debug, Default, Resource)]
pub struct LoadMap {
    /// The track to load.
    pub map: Handle<Scene>,
}

/// The instance of the map.
///
/// All parts of the map are children of this instance.
#[derive(Clone, Component, Debug, Default)]
pub struct MapInstance;

/// Generates collision for a mesh.
#[derive(Clone, Component, Debug, Default)]
pub struct GenerateTriMesh;

fn wait_for_assets(
    mut commands: Commands,
    mut next_state: ResMut<NextState<GameState>>,
    load_map: Res<LoadMap>,
) {
    commands.spawn((
        SceneBundle {
            scene: load_map.map.clone(),
            ..default()
        },
        MapInstance,
    ));

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::WHITE,
            illuminance: 10000.,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::default().looking_to(Vec3::new(0.25, -1., 0.25).normalize(), Vec3::Y),
        ..default()
    });

    next_state.set(GameState::InGame);
}

fn add_generate_tri_meshes(
    mut commands: Commands,
    parents_query: Query<&Parent>,
    map_instance_query: Query<(), With<MapInstance>>,
    meshes_query: Query<Entity, With<Handle<Mesh>>>,
) {
    for mesh_entity in meshes_query.iter() {
        for parent in parents_query.iter_ancestors(mesh_entity) {
            if map_instance_query.contains(parent) {
                commands.entity(mesh_entity).insert(GenerateTriMesh);
            }
        }
    }
}

fn generate_tri_meshes(
    mut commands: Commands,
    generate_query: Query<(Entity, &Handle<Mesh>), (With<GenerateTriMesh>, Without<Collider>)>,
    meshes: Res<Assets<Mesh>>,
) {
    for (entity, mesh) in generate_query.iter() {
        let Some(mesh) = meshes.get(mesh) else {
            continue;
        };

        // use mesh to generate a collider
        let Some(vertices) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else {
            continue;
        };

        let VertexAttributeValues::Float32x3(vertices) = vertices else {
            continue;
        };

        let vertices = vertices
            .iter()
            .map(|[x, y, z]| Vec3::new(*x, *y, *z))
            .collect::<Vec<_>>();

        let Some(indices) = mesh.indices() else {
            continue;
        };

        let indices = match mesh.primitive_topology() {
            PrimitiveTopology::TriangleList => match indices {
                Indices::U16(indices) => triangle_list(indices),
                Indices::U32(indices) => triangle_list(indices),
            },
            PrimitiveTopology::TriangleStrip => match indices {
                Indices::U16(indices) => triangle_strip(indices),
                Indices::U32(indices) => triangle_strip(indices),
            },
            _ => continue,
        };

        commands
            .entity(entity)
            .insert(Collider::trimesh(vertices, indices));
    }
}

fn triangle_list<T>(slice: &[T]) -> Vec<[u32; 3]>
where
    T: Copy + Into<u32>,
{
    slice
        .chunks_exact(3)
        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
        .map(|[c1, c2, c3]| [c1.into(), c2.into(), c3.into()])
        .collect::<Vec<_>>()
}

fn triangle_strip<T>(slice: &[T]) -> Vec<[u32; 3]>
where
    T: Copy + Into<u32>,
{
    slice
        .windows(3)
        .step_by(2)
        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
        .map(|[c1, c2, c3]| [c1.into(), c2.into(), c3.into()])
        .collect::<Vec<_>>()
}
