//! 3D physics-based movement controller.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::GamePhase;

/// Plugin for this module.
pub struct ControllerPlugin;

impl Plugin for ControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PreUpdate, update_jump_height).add_systems(
            FixedUpdate,
            (
                find_contact_normals,
                apply_controller_movement,
                reset_controllers,
            )
                .chain()
                .in_set(GamePhase::Movement),
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
    pub ccd: Ccd,
    pub friction: Friction,
    pub velocity: Velocity,
    pub external_force: ExternalForce,
    pub external_impulse: ExternalImpulse,
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
            ccd: Ccd::enabled(),
            friction: Friction {
                coefficient: 0.0, // LOL
                combine_rule: CoefficientCombineRule::Min,
            },
            velocity: default(),
            external_force: default(),
            external_impulse: default(),
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
    /// The threshold a contact point must meet for the controller to be
    /// considered grounded.
    pub ground_threshold: f32,
}

impl Default for ControllerOptions {
    fn default() -> Self {
        ControllerOptions {
            gravity: Vec3::new(0.0, -9.81 * 8., 0.0),
            ground_threshold: 0.6,
        }
    }
}

/// A component that actually holds the state of the controller.
#[derive(Clone, Component, Debug)]
pub struct Controller {
    movement: Vec2,
    y_impulse: f32,

    grounded: bool,
    contact_normal: Vec3,
}

impl Default for Controller {
    fn default() -> Self {
        Controller {
            movement: Vec2::ZERO,
            y_impulse: 0.0,

            grounded: false,
            contact_normal: Vec3::Y,
        }
    }
}

impl Controller {
    /// Checks if the controller is grounded.
    pub fn grounded(&self) -> bool {
        self.grounded
    }

    /// Projects a vector onto the normal plane.
    pub fn project_on_contact_plane(&self, vec: Vec3) -> Vec3 {
        vec - self.contact_normal * vec.dot(self.contact_normal)
    }

    /// Will attempt to move the controller in the general XZ direction of the
    /// passed [`Vec2`] for the frame.
    ///
    /// The vector passed is the "desired speed" in units/sec. The controller
    /// will attempt to accelerate and not overshoot.
    pub fn update_move(&mut self, movement: Vec2) {
        self.movement = movement;
    }

    /// Sets the y-impulse for the frame.
    ///
    /// This is of course done to allow controllers to jump.
    pub fn update_y_impulse(&mut self, y_impulse: f32) {
        self.y_impulse = y_impulse;
    }

    fn reset_y_impulse(&mut self) {
        self.y_impulse = 0.0;
    }
}

/// A component to describe movement speed.
///
/// This is not directly used by anything in this module, but can be used as a
/// cooperative component for all speed related functions.
#[derive(Clone, Copy, Component, Debug, Default)]
pub struct Speed(pub f32);

/// A component to describe jump height.
#[derive(Clone, Copy, Component, Debug, Default)]
pub struct JumpHeight {
    height: f32,
    impulse: f32,
}

impl JumpHeight {
    /// Creates a new [`JumpHeight`].
    ///
    /// # Panics
    /// Panics if the height is negative.
    pub fn new(height: f32) -> JumpHeight {
        assert!(height >= 0.);

        JumpHeight {
            height,
            impulse: 0.0,
        }
    }

    /// Gets the impulse for the jump.
    pub fn impulse(&self) -> f32 {
        self.impulse
    }
}

/// Updates the [`JumpHeight`] component for when it gets added, or the
/// controller attached gets changed.
pub fn update_jump_height(
    mut jump_height: Query<(&mut JumpHeight, &ControllerOptions), Changed<ControllerOptions>>,
) {
    for (mut jump_height, opts) in jump_height.iter_mut() {
        jump_height.impulse = (2. * jump_height.height * opts.gravity.length()).sqrt();
    }
}

/// This system finds contact normals for controllers.
pub fn find_contact_normals(
    rapier_ctx: Res<RapierContext>,
    mut controllers: Query<(Entity, &mut Controller, &ControllerOptions)>,
) {
    for (entity, mut con, copts) in controllers.iter_mut() {
        // get gravity normal
        // TODO: cache this normal
        let gravity_normal = -copts.gravity.normalize();

        let mut total_normal_count = 0;
        let mut total_normal_sum = Vec3::ZERO;

        // get all contact normals
        for contact_pair in rapier_ctx.contact_pairs_with(entity) {
            if contact_pair.has_any_active_contacts() {
                // iterate through active contacts
                for manifold in contact_pair.manifolds() {
                    let normal = if Some(entity) == manifold.rigid_body1() {
                        Some(manifold.local_n2())
                    } else if Some(entity) == manifold.rigid_body2() {
                        Some(manifold.local_n1())
                    } else {
                        None
                    };

                    if let Some(normal) = normal {
                        if normal.length_squared() > std::f32::EPSILON {
                            // check if this is grounded
                            let up_factor = normal.dot(gravity_normal);

                            if up_factor >= copts.ground_threshold {
                                // add to normal count
                                total_normal_count += 1;
                                total_normal_sum += normal;
                            }
                        }
                    }
                }
            }
        }

        // check if we got any ground contact normals
        if total_normal_count > 0 {
            con.grounded = true;
            con.contact_normal = total_normal_sum / (total_normal_count as f32);
        } else {
            con.grounded = false;
            con.contact_normal = gravity_normal;
        }
    }
}

/// Actually applies the controller movement.
pub fn apply_controller_movement(
    mut controllers: Query<(
        &Controller,
        &ControllerOptions,
        &Velocity,
        &mut ExternalForce,
        &mut ExternalImpulse,
    )>,
    time: Res<Time<Fixed>>,
) {
    for (con, copts, vel, mut ef, mut ei) in controllers.iter_mut() {
        // reset directional ef
        ef.force = Vec3::ZERO;

        // project current velocity onto plane to get right and forward on the
        // plane
        let right = con.project_on_contact_plane(Vec3::X).normalize();
        let forward = con.project_on_contact_plane(Vec3::Z).normalize();

        // get current velocity
        let right_vel = right.dot(vel.linvel);
        let forward_vel = forward.dot(vel.linvel);

        // calculate force required to realign the velocity
        let right_force = calculate_force(&*time, right_vel, con.movement.x);
        let forward_force = calculate_force(&*time, forward_vel, con.movement.y);

        // unproject to create force
        let force = right_force * right + forward_force * forward;
        ef.force += force;

        // apply gravity force
        if !con.grounded() {
            ef.force += copts.gravity;
        }

        // apply y-impulse in direction of contact plane
        ei.impulse += Vec3::new(0., con.y_impulse, 0.);
    }
}

/// Resets controllers for next frame.
pub fn reset_controllers(mut controllers: Query<&mut Controller>) {
    for mut con in controllers.iter_mut() {
        con.reset_y_impulse();
    }
}

fn calculate_force(time: &Time<Fixed>, current: f32, desired: f32) -> f32 {
    // find difference between desired velocity
    let diff = desired - current;

    // calculate an [`ExternalForce`] that will accelerate it to the speed
    // in the next timestep
    diff / time.delta_seconds()
}
