use bevy::{ecs::entity::MapEntities, prelude::*, reflect};

use crate::physics::rigidbody::RigidBodyItem;

use super::xpbd_constraint::*;

#[derive(Component, Clone, Copy)]
// #[reflect(Component, MapEntities)]
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

impl MapEntities for DistanceJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}

impl XPBDConstraint for DistanceJoint {
    fn entities(&self) -> [&Entity; 2] {
        [&self.entity1, &self.entity2]
    }
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
