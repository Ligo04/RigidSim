use bevy::{ecs::entity::MapEntities, prelude::*};

use crate::physics::rigidbody::RigidBodyQueryItem;

use super::xpbd_constraint::*;

#[derive(Component, Clone, Copy, Debug)]
pub struct DistanceJoint {
    pub entity1: Entity,
    pub entity2: Entity,
    pub local_anchor1: Vec3,      // local anchor point in entity1
    pub local_anchor2: Vec3,      // local anchor point in entity2
    pub rest_length: f32,         // rest length of the distance joint
    pub lagrange_multiplier: f32, // lagrange multiplier
    pub compliance: f32,          // compliance
    pub force: Vec3,              // exert force (joint)
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
    fn solve(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32) {
        self.force = self.solve_constraint(rigid_bodys, dt)
    }
}

impl XPBDPositionConstraint for DistanceJoint {}

impl DistanceJoint {
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1: entity1,
            entity2: entity2,
            local_anchor1: Vec3::ZERO,
            local_anchor2: Vec3::ZERO,
            rest_length: 0.0,
            lagrange_multiplier: 0.0,
            compliance: 0.0,
            force: Vec3::ZERO,
        }
    }

    pub fn set_anchor1(self, anchor: Vec3) -> Self {
        Self {
            local_anchor1: anchor,
            ..self
        }
    }

    pub fn set_anchor2(self, anchor: Vec3) -> Self {
        Self {
            local_anchor2: anchor,
            ..self
        }
    }

    pub fn set_rest_length(self, rest_length: f32) -> Self {
        Self {
            rest_length: rest_length,
            ..self
        }
    }

    pub fn set_compliance(self, compliance: f32) -> Self {
        Self {
            compliance: compliance,
            ..self
        }
    }

    pub fn set_lagrange_multiplier(self, lagrange_multiplier: f32) -> Self {
        Self {
            lagrange_multiplier: lagrange_multiplier,
            ..self
        }
    }

    pub fn set_force(self, force: Vec3) -> Self {
        Self {
            force: force,
            ..self
        }
    }

    fn solve_constraint(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32) -> Vec3 {
        let [body1, body2] = rigid_bodys;
        let world_r1 = body1.curr_transform.0.rotation * self.local_anchor1;
        let world_r2 = body2.curr_transform.0.rotation * self.local_anchor2;

        let p1 = body1.get_world_curr_position() + world_r1;
        let p2 = body2.get_world_curr_position() + world_r2;
        let (normal, c) = self.compute_correction_force(p1, p2);

        if c <= f32::EPSILON {
            return Vec3::ZERO;
        }

        let body1_w = <DistanceJoint as XPBDPositionConstraint>::compute_generalized_inverse_mass(
            body1, normal, world_r1,
        );
        let body2_w = <DistanceJoint as XPBDPositionConstraint>::compute_generalized_inverse_mass(
            body2, normal, world_r2,
        );
        let delta_lagrange = self.compute_detla_lagrange(
            self.lagrange_multiplier,
            body1_w,
            body2_w,
            c,
            self.compliance,
            dt,
        );
        self.lagrange_multiplier += delta_lagrange;
        self.apply_positional_correction(body1, body2, world_r1, world_r2, delta_lagrange, normal);
        self.compute_force(self.lagrange_multiplier, normal, dt)
    }

    // compute distance joint correction force
    fn compute_correction_force(&self, pos1: Vec3, pos2: Vec3) -> (Vec3, f32) {
        // TODO: update this function
        let delta = pos2 - pos1;
        let dist = delta.length();
        let correction = (dist - self.rest_length) / dist;
        let force = delta * correction;
        (force.normalize(), force.length())
    }
}
