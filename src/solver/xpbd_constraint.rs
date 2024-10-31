use bevy::{
    ecs::entity::MapEntities,
    math::{Quat, Vec3},
    prelude::Entity,
};

use crate::physics::rigidbody::RigidBodyItem;

pub trait XPBDConstraint: MapEntities {
     fn entities(&self) -> [&Entity; 2];
     fn clear_lagrange_multiplier(&mut self);
     fn solve(&mut self, rigid_bodys: [&mut RigidBodyItem; 2], dt: f32);

     fn compute_detla_lagrange(
        &self,
        lagrance: f32,
        w1: f32,
        w2: f32,
        c: f32,
        compliance: f32,
        dt: f32,
    ) -> f32 {
        let w_sum = w1 + w2;
        if w_sum <= f32::EPSILON {
            return 0.0;
        }
        // compliance = compliance / h^2
        let tilde_compliance = compliance / dt.powi(2);
        (-c - tilde_compliance * lagrance) / (w_sum + tilde_compliance)
    }
}

pub trait XPBDPositionConstraint: XPBDConstraint {
    fn apply_positional_correction(
        &self,
        body1: &mut RigidBodyItem,
        body2: &mut RigidBodyItem,
        r1: Vec3,
        r2: Vec3,
        delta_lagrange: f32,
        normal: Vec3,
    ) -> Vec3 {
        if delta_lagrange.abs() <= f32::EPSILON {
            return Vec3::ZERO;
        }

        let impulse = delta_lagrange * normal;
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
            let mut delta_q = Quat::from_scaled_axis(inv_inertia1.inverse() * r1.cross(impulse));
            delta_q = (delta_q * body1.accu_transform.rotation) * 0.5;
            body1.accu_transform.rotation = body1.accu_transform.rotation + delta_q;
        }
        if body2.rigid_type.is_dynamic() {
            // position
            body2.accu_transform.translation -= inv_mass2 * impulse;
            // rotation
            let mut delta_q = Quat::from_scaled_axis(inv_inertia2.inverse() * r2.cross(impulse));
            delta_q = (delta_q * body2.accu_transform.rotation) * 0.5;
            body1.accu_transform.rotation = body2.accu_transform.rotation - delta_q;
        }

        impulse
    }

    fn compute_force(&self, lagrange: f32, normal: Vec3, dt: f32) -> Vec3 {
        lagrange * normal / dt.powi(2)
    }
}

//TODO: implement this trait(Angular Constraint)
pub trait XPBDAngularConstraint: XPBDConstraint {}
