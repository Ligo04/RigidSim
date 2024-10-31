use bevy::prelude::*;

use crate::physics::rigidbody::RigidBodyItem;

use super::xpbd_constraint::*;

#[derive(Component, Clone, Copy)]
pub struct DistanceJoint {
    pub entity1: Entity,
    pub entity2: Entity,
    pub local_anchor1: Vec3,      // local anchor point in entity1
    pub local_anchor2: Vec3,      // local anchor point in entity2
    pub rest_length: f32,         // rest length of the distance joint
    pub lagrange_multiplier: f32, // lagrange multiplier
    pub compliance: f32,          // compliance
    pub exert_force: Vec3,        // exert force (joint)
}

impl XPBDConstraint for DistanceJoint {
    fn clear_lagrange_multiplier(&mut self) {
        self.lagrange_multiplier = 0.0;
    }
    fn solve(&mut self, rigid_bodys: [&mut RigidBodyItem; 2], dt: f32) {
        self.exert_force = self.solve_constraint(rigid_bodys, dt)
    }
}

impl DistanceJoint {
    fn solve_constraint(&mut self, rigid_bodys: [&mut RigidBodyItem; 2], dt: f32) -> Vec3 {
        let [body1, body2] = rigid_bodys;
        //TODO: implement this function
        Vec3::ZERO
    }
}
