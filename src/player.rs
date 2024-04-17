//! The local player, the local camera, and other interfaces.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::physics::controller::ControllerBundle;
use crate::{GamePhase, GameState};

/// Contains systems for the player and the camera.
pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(GameState::InGame), spawn_player)
            .add_systems(
                Update,
                (
                    bevy::transform::systems::propagate_transforms,
                    update_camera_position,
                )
                    .chain()
                    .in_set(GamePhase::Camera)
                    .after(GamePhase::Movement),
            );
    }
}

/// A marker component that marks the local player.
///
/// There can only be one local player at a time.
#[derive(Clone, Copy, Component, Debug, Default)]
pub struct LocalPlayer;

/// A marker component that marks the local camera.
///
/// Similarly to [`LocalPlayer`], there can only be one local camera at a time.
#[derive(Clone, Copy, Component, Debug, Default)]
pub struct LocalCamera;

/// A system that spawns the player in after loading.
pub fn spawn_player(mut commands: Commands) {
    // spawn the player
    let _player = commands.spawn((
        ControllerBundle {
            transform: Transform::from_xyz(0.0, 3.0, 0.0),
            ..default()
        },
        LocalPlayer,
    ));

    // spawn the camera
    commands.spawn((Camera3dBundle::default(), LocalCamera));
}

/// Updates the camera's movement based on the player position.
pub fn update_camera_position(
    player: Query<&GlobalTransform, With<LocalPlayer>>,
    mut camera: Query<&mut Transform, With<LocalCamera>>,
) {
    let (Ok(player_transform), Ok(mut camera_transform)) =
        (player.get_single(), camera.get_single_mut())
    else {
        return;
    };

    // sync positions
    camera_transform.translation = player_transform.translation();
}
