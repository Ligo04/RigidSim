use bevy::{
    ecs::entity::{Entities, MapEntities},
    math::Vec3,
    prelude::Entity,
};

use crate::physics::rigidbody::RigidBodyItem;

pub trait XPBDConstraint: MapEntities {
    fn entities(&self) -> [&Entity; 2];
    fn clear_lagrange_multiplier(&mut self);
    fn solve(&mut self, rigid_bodys: [&mut RigidBodyItem; 2], dt: f32);
}

pub trait XPBDPositionConstraint: XPBDConstraint {
    fn apply_positional_correction(
        &self,
        body1: &mut RigidBodyItem,
        body2: &mut RigidBodyItem,
        r1: Vec3,
        r2: Vec3,
        delta_lagrange: f32,
        direction: Vec3,
    ) -> Vec3 {
        if delta_lagrange.abs() <= f32::EPSILON {
            return Vec3::ZERO;
        }

        let impulse = delta_lagrange * direction;
        self.apply_position_impulse(body1, body2, r1, r2, impulse)
    }

    fn apply_position_impulse(
        &self,
        body1: &mut RigidBodyItem,
        body2: &mut RigidBodyItem,
        r1: Vec3,
        r2: Vec3,
        impulse: Vec3,
    ) -> Vec3 {
        let inv_mass1 = body1.mass.inverse();
        let inv_mass2 = body2.mass.inverse();
        let inv_inertia1 = body1.compute_world_inv_interia();
        let inv_inertia2 = body2.compute_world_inv_interia();

        if body1.rigid_type.is_dynamic() {
            // position
            body1.accu_transform.translation += inv_mass1 * impulse;
            // rotation
        }
        if body2.rigid_type.is_dynamic() {
            // position
            body2.accu_transform.translation += inv_mass2 * impulse;
            // rotation
        }

        impulse
    }
}

pub trait XPBDAngularConstraint: XPBDConstraint {}
