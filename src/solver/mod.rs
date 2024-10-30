pub mod distance_joint;
pub mod xpbd_constraint;

use bevy::prelude::*;
pub struct XpbdSolverPlugin;

impl Plugin for XpbdSolverPlugin {
    fn build(&self, app: &mut App) {}
}

fn substep() {}
