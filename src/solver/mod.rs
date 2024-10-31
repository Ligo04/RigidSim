pub mod distance_joint;
pub mod xpbd_constraint;

use bevy::prelude::*;
use xpbd_constraint::XPBDConstraint;

use crate::physics::rigidbody::RigidBody;

#[derive(Resource, Clone, Copy)]
pub struct SubStepCount(pub u32);

impl Default for SubStepCount {
    fn default() -> Self {
        Self(5)
    }
}

pub struct XpbdSolverPlugin;

impl Plugin for XpbdSolverPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(SubStepCount::default());
    }
}

fn substep(
    time: Res<Time>,
    sub_step_count: Res<SubStepCount>,
    mut commands: Commands,
    mut bodies: Query<RigidBody>,
) {
    println!("substep");
    let dt = time.delta_seconds_f64() / sub_step_count.0 as f64;
    for i in 0..sub_step_count.0 {
        println!("substep {}", i);
    }
}
