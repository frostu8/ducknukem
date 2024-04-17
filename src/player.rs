//! The local player, the local camera, and other interfaces.

use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, PrimaryWindow};
use bevy_rapier3d::prelude::*;

use std::time::Duration;

use crate::physics::controller::{Controller, ControllerBundle, JumpHeight, Speed};
use crate::{GamePhase, GameState};

/// A factor applied with the sensitivity that roughly translates logical units
/// to radians.
pub const CAMERA_MOUSE_FACTOR: f32 = 0.006;
/// Range to lock pitch.
pub const PITCH_LOCK: f32 = 75. * (std::f32::consts::PI / 180.);

/// Contains systems for the player and the camera.
pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LocalPlayerOptions>()
            .add_systems(OnEnter(GameState::InGame), spawn_player)
            .add_systems(
                Update,
                (
                    process_camera_lock_input,
                    process_camera_input,
                    process_movement_input,
                    unbuffer_jump_timer,
                )
                    .chain()
                    .in_set(GamePhase::Input),
            )
            .add_systems(
                Update,
                (
                    bevy::transform::systems::propagate_transforms,
                    lock_mouse,
                    update_camera_position,
                )
                    .chain()
                    .in_set(GamePhase::Camera)
                    .after(GamePhase::Movement)
                    .after(GamePhase::Input),
            );
    }
}

/// A resource for local player options.
#[derive(Clone, Debug, Resource)]
pub struct LocalPlayerOptions {
    pub y_sensitivity: f32,
    pub x_sensitivity: f32,
}

impl Default for LocalPlayerOptions {
    fn default() -> Self {
        LocalPlayerOptions {
            y_sensitivity: 1.0,
            x_sensitivity: 1.0,
        }
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
#[derive(Clone, Copy, Component, Debug)]
pub struct LocalCamera {
    mouse_locked: bool,
    yaw: f32,
    pitch: f32,
}

impl Default for LocalCamera {
    fn default() -> Self {
        LocalCamera {
            mouse_locked: true,
            yaw: 0.0,
            pitch: 0.0,
        }
    }
}

impl LocalCamera {
    /// Gets the normalized forward vector in the XZ local space.
    pub fn forward(&self) -> Vec2 {
        Vec2::new(self.yaw.sin(), self.yaw.cos())
    }

    /// Gets the normalized right vector in the XZ local space.
    pub fn right(&self) -> Vec2 {
        Vec2::new(-self.yaw.cos(), self.yaw.sin())
    }
}

/// Jump timer things.
///
/// This serves as a sort of buffer for jump presses.
#[derive(Clone, Component, Debug)]
pub struct JumpTimer {
    buffer_time: Duration,
    timer: Timer,
    ticking: bool,
}

impl JumpTimer {
    /// Creates a new jump timer.
    pub fn new(buffer_time: Duration) -> JumpTimer {
        JumpTimer {
            buffer_time,
            timer: Timer::new(buffer_time, TimerMode::Once),
            ticking: false,
        }
    }

    /// Checks if the window is still in.
    pub fn running(&self) -> bool {
        self.ticking && !self.timer.finished()
    }

    /// Sets the jump timer.
    pub fn set(&mut self) {
        self.ticking = true;
        self.timer = Timer::new(self.buffer_time, TimerMode::Once);
    }

    /// Stops the jump timer.
    pub fn stop(&mut self) {
        self.ticking = false;
    }

    fn tick(&mut self, delta: Duration) {
        self.timer.tick(delta);
    }
}

/// A system that spawns the player in after loading.
pub fn spawn_player(mut commands: Commands) {
    // spawn the player
    let _player = commands.spawn((
        ControllerBundle {
            transform: Transform::from_xyz(0.0, 3.0, 0.0),
            ..default()
        },
        Speed(15.),
        JumpHeight::new(6.),
        JumpTimer::new(Duration::from_millis(55)),
        LocalPlayer,
    ));

    // spawn the camera
    commands.spawn((Camera3dBundle::default(), LocalCamera::default()));
}

/// Updates the camera's pitch and yaw from cursor movement.
pub fn process_camera_input(
    mut camera: Query<&mut LocalCamera>,
    mut mouse_motion_rx: EventReader<MouseMotion>,
    opts: Res<LocalPlayerOptions>,
) {
    let Ok(mut local_camera) = camera.get_single_mut() else {
        return;
    };

    if local_camera.mouse_locked {
        for motion in mouse_motion_rx.read() {
            // this must be negative because it rotates counterclockwise
            local_camera.yaw += -motion.delta.x * opts.x_sensitivity * CAMERA_MOUSE_FACTOR;
            local_camera.pitch += -motion.delta.y * opts.y_sensitivity * CAMERA_MOUSE_FACTOR;

            // lock pitch between around 75 and -75 degrees
            local_camera.pitch = local_camera.pitch.clamp(-PITCH_LOCK, PITCH_LOCK);
        }
    }
}

/// Unlocks and relocks the mouse from user input.
pub fn process_camera_lock_input(
    mut camera: Query<&mut LocalCamera>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mouse_input: Res<ButtonInput<MouseButton>>,
) {
    let Ok(mut local_camera) = camera.get_single_mut() else {
        return;
    };

    if local_camera.mouse_locked {
        // listen for escape events
        if keyboard_input.just_pressed(KeyCode::Escape) {
            local_camera.mouse_locked = false;
        }
    } else {
        // listen for mouse events.
        if mouse_input.just_pressed(MouseButton::Left) {
            local_camera.mouse_locked = true;
        }
    }
}

/// Processes the movement input from the player.
pub fn process_movement_input(
    mut player: Query<(&mut Controller, &Speed, &mut JumpTimer), With<LocalPlayer>>,
    camera: Query<&LocalCamera>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    let (Ok((mut controller, speed, mut jump_timer)), Ok(local_camera)) =
        (player.get_single_mut(), camera.get_single())
    else {
        return;
    };

    // TODO: input mapping :(
    let axis_forward = if keyboard_input.pressed(KeyCode::KeyW) {
        -1.
    } else if keyboard_input.pressed(KeyCode::KeyS) {
        1.
    } else {
        0.
    };
    let axis_right = if keyboard_input.pressed(KeyCode::KeyD) {
        -1.
    } else if keyboard_input.pressed(KeyCode::KeyA) {
        1.
    } else {
        0.
    };

    let forward = local_camera.forward() * axis_forward * speed.0;
    let right = local_camera.right() * axis_right * speed.0;

    controller.update_move(forward + right);

    if keyboard_input.just_pressed(KeyCode::Space) {
        jump_timer.set();
    }
}

/// Actually sets the jump impulse when a jump is detected.
pub fn unbuffer_jump_timer(
    mut query: Query<(&mut JumpTimer, &JumpHeight, &mut Controller)>,
    time: Res<Time>,
) {
    for (mut jump_timer, jump_height, mut controller) in query.iter_mut() {
        if jump_timer.running() && controller.grounded() {
            controller.update_y_impulse(jump_height.impulse());

            // unset jump timer
            jump_timer.stop();
        }

        jump_timer.tick(time.delta());
    }
}

/// Updates the cursor lock from the [`LocalCamera`] interface.
pub fn lock_mouse(
    created_camera: Query<&LocalCamera, Changed<LocalCamera>>,
    mut primary_window: Query<&mut Window, With<PrimaryWindow>>,
) {
    let Ok(mut primary_window) = primary_window.get_single_mut() else {
        return;
    };

    for local_camera in created_camera.iter() {
        if local_camera.mouse_locked {
            primary_window.cursor.grab_mode = CursorGrabMode::Locked;
            primary_window.cursor.visible = false;
        } else {
            primary_window.cursor.grab_mode = CursorGrabMode::None;
            primary_window.cursor.visible = true;
        }
    }
}

/// Updates the camera's movement based on the player position.
pub fn update_camera_position(
    player: Query<&GlobalTransform, With<LocalPlayer>>,
    mut camera: Query<(&mut Transform, &LocalCamera)>,
) {
    let (Ok(player_transform), Ok((mut camera_transform, local_camera))) =
        (player.get_single(), camera.get_single_mut())
    else {
        return;
    };

    // sync positions
    camera_transform.translation = player_transform.translation();

    // apply pitch and yaw
    camera_transform.rotation =
        Quat::from_rotation_y(local_camera.yaw) * Quat::from_rotation_x(local_camera.pitch);
}
