use bevy::{
    math::{Quat, Vec3},
    prelude::Entity,
};

use crate::physics::rigidbody::RigidBodyQueryItem;

pub trait XPBDConstraint {
    fn entities(&self) -> [Entity; 2];
    fn clear_lagrange_multiplier(&mut self);
    fn solve(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32);
    fn solve_joint_damping(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32);
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
        if w_sum.abs() <= f32::EPSILON {
            return 0.0;
        }
        // compliance = compliance / h^2
        let total_compliance = compliance / dt.powi(2);
        // delta_lagrange = (-c - total_compliance * lagrance) / (w_sum + total_compliance)
        (-c - total_compliance * lagrance) / (w_sum + total_compliance)
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
        let inv_mass1 = body1.get_inv_mass();
        let inv_mass2 = body2.get_inv_mass();
        let inv_inertia1 = body1.compute_world_inv_interia();
        let inv_inertia2 = body2.compute_world_inv_interia();

        if body1.rigid_type.is_dynamic() {
            // position  x1  = x1 + p / m1
            body1.curr_transform.translation += inv_mass1 * impulse;
            // rotation
            // delta_w = 0.5 * inv_inertia1 * r1.cross(impulse)
            // q <- q + 0.5 * [delta_w,0] * q
            let delta_w = 0.5 * inv_inertia1 * r1.cross(impulse);
            let delta_q = Quat::from_vec4(delta_w.extend(0.0)) * body1.curr_transform.rotation;
            body1.curr_transform.rotation = (body1.curr_transform.rotation + delta_q).normalize();
        }
        if body2.rigid_type.is_dynamic() {
            // position
            body2.curr_transform.translation -= inv_mass2 * impulse;
            // rotation
            // q <- q - 0.5 * [delta_w,0] * q
            let delta_w = 0.5 * inv_inertia2 * r2.cross(impulse);
            let delta_q = Quat::from_vec4(delta_w.extend(0.0)) * body2.curr_transform.rotation;
            body2.curr_transform.rotation = (body2.curr_transform.rotation - delta_q).normalize();
        }

        impulse
    }

    fn compute_force(&self, lagrange: f32, normal: Vec3, dt: f32) -> Vec3 {
        lagrange * normal / dt.powi(2)
    }

    fn compute_generalized_inverse_mass(
        &self,
        body: &RigidBodyQueryItem,
        normal: Vec3,
        r: Vec3,
    ) -> f32 {
        if body.rigid_type.is_dynamic() {
            let rn: Vec3 = r.cross(normal);
            let inv_interia = body.compute_world_inv_interia();
            body.get_inv_mass() + rn.dot(inv_interia * rn)
        } else {
            0.0
        }
    }
}

pub trait XPBDAngularConstraint: XPBDConstraint {
    fn compute_generalized_inverse_mass(body: &RigidBodyQueryItem, normal: Vec3) -> f32 {
        if body.rigid_type.is_dynamic() {
            let inv_interia = body.compute_world_inv_interia();
            normal.dot(inv_interia * normal)
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
            // delta_w = 0.5 * inv_inertia1 * r1.cross(impulse)
            // q <- q + 0.5 * [delta_w,0] * q
            let delta_w = 0.5 * inv_inertia1 * r1.cross(impulse);
            let delta_q = Quat::from_vec4(delta_w.extend(0.0)) * body1.curr_transform.rotation;
            body1.curr_transform.rotation = (body1.curr_transform.rotation + delta_q).normalize();
        }
        if body2.rigid_type.is_dynamic() {
            // rotation
            // q <- q - 0.5 * [delta_w,0] * q
            let delta_w = 0.5 * inv_inertia2 * r2.cross(impulse);
            let delta_q = Quat::from_vec4(delta_w.extend(0.0)) * body2.curr_transform.rotation;
            body2.curr_transform.rotation = (body2.curr_transform.rotation - delta_q).normalize();
        }
        impulse
    }

    fn compute_torque(&self, lagrange: f32, normal: Vec3, dt: f32) -> Vec3 {
        lagrange * normal / dt.powi(2)
    }
}
