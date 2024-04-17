//! 3D physics-based movement controller.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::GamePhase;

/// Plugin for this module.
pub struct ControllerPlugin;

impl Plugin for ControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate,
            apply_controller_movement.in_set(GamePhase::Movement),
        );
    }
}

/// A bundle that allows an entity to act with a controller.
///
/// The default values are preferred. [`LockedAxes`] should **always** be
/// [`LockedAxes::ROTATION_LOCKED`].
#[derive(Bundle)]
pub struct ControllerBundle {
    pub transform: Transform,
    pub global_transform: GlobalTransform,
    pub visibility: Visibility,
    pub view_visibility: ViewVisibility,
    pub inherited_visibility: InheritedVisibility,
    pub collider: Collider,
    pub rigid_body: RigidBody,
    pub locked_axes: LockedAxes,
    pub gravity_scale: GravityScale,
    pub external_force: ExternalForce,
    pub controller_options: ControllerOptions,
    pub controller: Controller,
}

impl Default for ControllerBundle {
    fn default() -> Self {
        ControllerBundle {
            transform: default(),
            global_transform: default(),
            visibility: default(),
            view_visibility: default(),
            inherited_visibility: default(),
            collider: Collider::capsule_y(1.0, 0.5),
            rigid_body: RigidBody::Dynamic,
            locked_axes: LockedAxes::ROTATION_LOCKED,
            gravity_scale: GravityScale(0.0),
            external_force: default(),
            controller_options: default(),
            controller: default(),
        }
    }
}

/// A component that holds controller options.
///
/// These can be set dynamically in-game.
#[derive(Clone, Component, Debug)]
pub struct ControllerOptions {
    /// The (typically down) gravity vector.
    pub gravity: Vec3,
}

impl Default for ControllerOptions {
    fn default() -> Self {
        ControllerOptions {
            gravity: Vec3::new(0.0, -9.81 * 4., 0.0),
        }
    }
}

/// A component that actually holds the state of the controller.
#[derive(Clone, Component, Debug)]
pub struct Controller {
    movement: Vec2,
}

impl Default for Controller {
    fn default() -> Self {
        Controller {
            movement: Vec2::ZERO,
        }
    }
}

impl Controller {
    /// Will attempt to move the controller in the general XZ direction of the
    /// passed [`Vec2`] for the frame.
    pub fn update_move(&mut self, movement: Vec2) {
        self.movement = movement;
    }
}

/// Actually applies the controller movement.
pub fn apply_controller_movement(
    mut controllers: Query<(&Controller, &ControllerOptions, &mut ExternalForce)>,
) {
    for (con, copts, mut ef) in controllers.iter_mut() {
        // reset directional ef
        ef.force = Vec3::ZERO;

        // apply gravity force
        ef.force += copts.gravity;
    }
}
