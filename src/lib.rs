//! Duck Game!

pub mod map;
pub mod physics;
pub mod player;

use bevy::app::PluginGroupBuilder;
use bevy::prelude::*;

/// The game's plugins.
pub struct GamePlugins;

impl PluginGroup for GamePlugins {
    fn build(self) -> PluginGroupBuilder {
        PluginGroupBuilder::start::<Self>()
            .add(physics::controller::ControllerPlugin)
            .add(player::PlayerPlugin)
            .add(map::MapPlugin)
            .add(GameStatePlugin)
    }
}

/// A plugin for the main game state.
pub struct GameStatePlugin;

impl Plugin for GameStatePlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<GameState>();
    }
}

/// The phases of the game logic.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, SystemSet)]
pub enum GamePhase {
    /// Input scanning.
    Input,
    /// Applying input in movement.
    Movement,
    /// For late-calculated camera shenanigans to prevent 1-frame delay.
    Camera,
}

/// The state of the game.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, States)]
pub enum GameState {
    /// Default state.
    #[default]
    Splash,
    /// Loading a map. See the [`map`] module high-level documentation.
    LoadingMap,
    /// In a track.
    InGame,
}
