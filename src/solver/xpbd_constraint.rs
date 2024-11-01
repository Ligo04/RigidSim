use bevy::{
    ecs::entity::MapEntities,
    math::{Quat, Vec3},
    prelude::Entity,
};

use crate::physics::rigidbody::RigidBodyQueryItem;

pub trait XPBDConstraint: MapEntities {
    fn entities(&self) -> [&Entity; 2];
    fn clear_lagrange_multiplier(&mut self);
    fn solve(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32);

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
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
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
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        r1: Vec3,
        r2: Vec3,
        impulse: Vec3,
    ) -> Vec3 {
        let inv_mass1 = body1.mass.inverse();
        let inv_mass2 = body2.mass.inverse();
        let inv_inertia1 = body1.compute_world_inv_interia();
        let inv_inertia2 = body2.compute_world_inv_interia();

        if body1.rigid_type.is_dynamic() {
            // position  x1  = x1 + p / m1
            body1.curr_transform.0.translation += inv_mass1 * impulse;
            // rotation
            let mut delta_q = Quat::from_scaled_axis(inv_inertia1.inverse() * r1.cross(impulse));
            delta_q = delta_q * 0.5 * body1.curr_transform.0.rotation;
            body1.curr_transform.0.rotation = body1.curr_transform.0.rotation + delta_q;
            body1.curr_transform.0.rotation = body1.curr_transform.0.rotation.normalize();
        }
        if body2.rigid_type.is_dynamic() {
            // position
            body2.curr_transform.0.translation -= inv_mass2 * impulse;
            // rotation
            let mut delta_q = Quat::from_scaled_axis(inv_inertia2.inverse() * r2.cross(impulse));
            delta_q = delta_q * 0.5 * body2.curr_transform.0.rotation;
            body2.curr_transform.0.rotation = body2.curr_transform.0.rotation - delta_q;
            body2.curr_transform.0.rotation = body2.curr_transform.0.rotation.normalize();
        }

        impulse
    }

    fn compute_force(&self, lagrange: f32, normal: Vec3, dt: f32) -> Vec3 {
        lagrange * normal / dt.powi(2)
    }

    fn compute_generalized_inverse_mass(body: &RigidBodyQueryItem, normal: Vec3, r: Vec3) -> f32 {
        if body.rigid_type.is_dynamic() {
            let rn: Vec3 = r.cross(normal);
            let interia = body.compute_world_inv_interia();
            body.mass.inverse() + rn.dot(interia.inverse() * rn)
        } else {
            0.0
        }
    }
}

//TODO: implement this trait(Angular Constraint)
pub trait XPBDAngularConstraint: XPBDConstraint {
    fn compute_generalized_inverse_mass(body: &RigidBodyQueryItem, normal: Vec3) -> f32 {
        if body.rigid_type.is_dynamic() {
            let interia = body.compute_world_inv_interia();
            normal.dot(interia.inverse() * normal)
        } else {
            0.0
        }
    }

    fn apply_angular_correction(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        r1: Vec3,
        r2: Vec3,
        delta_lagrange: f32,
        normal: Vec3,
    ) -> Vec3 {
        if delta_lagrange.abs() <= f32::EPSILON {
            return Vec3::ZERO;
        }

        let impulse = delta_lagrange * normal;
        self.apply_angular_impluse(body1, body2, r1, r2, impulse)
    }

    fn apply_angular_impluse(
        &self,
        body1: &mut RigidBodyQueryItem,
        body2: &mut RigidBodyQueryItem,
        r1: Vec3,
        r2: Vec3,
        impulse: Vec3,
    ) -> Vec3 {
        let inv_inertia1 = body1.compute_world_inv_interia();
        let inv_inertia2 = body2.compute_world_inv_interia();

        //TODO: implement this
        if body1.rigid_type.is_dynamic() {
            // rotation
            let mut delta_q = Quat::from_scaled_axis(inv_inertia1.inverse() * r1.cross(impulse));
            delta_q = delta_q * 0.5 * body2.curr_transform.0.rotation;
            body1.curr_transform.0.rotation = body1.curr_transform.0.rotation + delta_q;
            body1.curr_transform.0.rotation = body1.curr_transform.0.rotation.normalize();
        }
        if body2.rigid_type.is_dynamic() {
            // rotation
            let mut delta_q = Quat::from_scaled_axis(inv_inertia2.inverse() * r2.cross(impulse));
            delta_q = delta_q * 0.5 * body2.curr_transform.0.rotation;
            body1.curr_transform.0.rotation = body1.curr_transform.0.rotation - delta_q;
            body1.curr_transform.0.rotation = body1.curr_transform.0.rotation.normalize();
        }
        impulse
    }

    fn compute_torque(&self, lagrange: f32, normal: Vec3, dt: f32) -> Vec3 {
        lagrange * normal / dt.powi(2)
    }

    fn project_linear_velocity(&self, body: &mut RigidBodyQueryItem, dt: f32) {
        // v = (x - x_prev) / h
        if body.rigid_type.is_dynamic() {
            let delta_vec =
                (body.curr_transform.0.translation - body.prev_transform.0.translation) / dt;
            body.velocity.0 = delta_vec;
        } else {
            body.velocity.0 = Vec3::ZERO;
        }
    }

    fn project_angular_velocity(&self, body: &mut RigidBodyQueryItem, dt: f32) {
        // delta_q = q * q_prev^-1
        // w = 2[delta_q_x,delta_q_y,delta_q_z] / h
        // w = delta_q>0 ? w : -w
        if body.rigid_type.is_dynamic() {
            let delta_q = body
                .curr_transform
                .0
                .rotation
                .mul_quat(body.prev_transform.0.rotation.inverse());
            let delta_w = 2.0 * delta_q.xyz() / dt;
            if delta_q.w > 0.0 {
                body.angular_velocity.0 += delta_w;
            } else {
                body.angular_velocity.0 -= delta_w;
            }
        } else {
            body.angular_velocity.0 = Vec3::ZERO;
        }
    }
}
