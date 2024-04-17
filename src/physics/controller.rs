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
            (find_contact_normals, apply_controller_movement)
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
    pub velocity: Velocity,
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
            velocity: default(),
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
    /// The threshold a contact point must meet for the controller to be
    /// considered grounded.
    pub ground_threshold: f32,
}

impl Default for ControllerOptions {
    fn default() -> Self {
        ControllerOptions {
            gravity: Vec3::new(0.0, -9.81 * 4., 0.0),
            ground_threshold: 0.6,
        }
    }
}

/// A component that actually holds the state of the controller.
#[derive(Clone, Component, Debug)]
pub struct Controller {
    movement: Vec2,
    grounded: bool,
    contact_normal: Vec3,
}

impl Default for Controller {
    fn default() -> Self {
        Controller {
            movement: Vec2::ZERO,
            grounded: false,
            contact_normal: Vec3::Y,
        }
    }
}

impl Controller {
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
}

/// A component to describe movement speed.
///
/// This is not directly used by anything in this module, but can be used as a
/// cooperative component for all speed related functions.
#[derive(Clone, Copy, Component, Debug, Default)]
pub struct Speed(pub f32);

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
                let (normal_sum, normal_count) = contact_pair
                    .manifolds()
                    .filter_map(|manifold| {
                        if Some(entity) == manifold.rigid_body1() {
                            Some(manifold.local_n2())
                        } else if Some(entity) == manifold.rigid_body2() {
                            Some(manifold.local_n1())
                        } else {
                            None
                        }
                    })
                    .filter(|n| n.length_squared() > std::f32::EPSILON)
                    .fold((Vec3::ZERO, 0), |(sum, count), x| (sum + x, count + 1));

                if normal_count > 0 {
                    let normal = normal_sum / (normal_count as f32);

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
    )>,
    time: Res<Time<Fixed>>,
) {
    for (con, copts, vel, mut ef) in controllers.iter_mut() {
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
        ef.force += copts.gravity;
    }
}

fn calculate_force(time: &Time<Fixed>, current: f32, desired: f32) -> f32 {
    // find difference between desired velocity
    let diff = desired - current;

    // calculate an [`ExternalForce`] that will accelerate it to the speed
    // in the next timestep
    diff / time.delta_seconds()
}
