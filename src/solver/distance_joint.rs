use bevy::prelude::*;

use crate::physics::rigidbody::RigidBodyQueryItem;

use super::xpbd_constraint::*;

pub trait Joint {
    fn local_anchor1(&self) -> Vec3;
    fn local_anchor2(&self) -> Vec3;
    fn velocity_damping(&self) -> f32;
    fn angular_damping(&self) -> f32;
    fn force(&self) -> Vec3;
}

#[derive(Component, Clone, Copy, Debug)]
pub struct DistanceJoint {
    pub entity1: Entity,
    pub entity2: Entity,
    pub local_anchor1: Vec3,      // local anchor point in entity1
    pub local_anchor2: Vec3,      // local anchor point in entity2
    pub rest_length: f32,         // rest length of the distance joint
    pub lagrange_multiplier: f32, // lagrange multiplier
    pub compliance: f32,          // compliance
    pub velocity_damping: f32,    // velocity damping
    pub angular_damping: f32,     // angular damping
    pub force: Vec3,              // exert force (joint)
}

impl Joint for DistanceJoint {
    fn local_anchor1(&self) -> Vec3 {
        self.local_anchor1
    }
    fn local_anchor2(&self) -> Vec3 {
        self.local_anchor2
    }
    fn velocity_damping(&self) -> f32 {
        self.velocity_damping
    }
    fn angular_damping(&self) -> f32 {
        self.angular_damping
    }
    fn force(&self) -> Vec3 {
        self.force
    }
}

impl XPBDConstraint for DistanceJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
    fn clear_lagrange_multiplier(&mut self) {
        self.lagrange_multiplier = 0.0;
    }
    fn solve(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32) {
        self.force = self.solve_constraint(rigid_bodys, dt)
    }

    fn solve_joint_damping(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32) {
        self.solve_joint_damping(rigid_bodys, dt)
    }
}

impl XPBDPositionConstraint for DistanceJoint {}

impl DistanceJoint {
    pub fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vec3::ZERO,
            local_anchor2: Vec3::ZERO,
            rest_length: 0.0,
            lagrange_multiplier: 0.0,
            compliance: 0.0,
            force: Vec3::ZERO,
            velocity_damping: 0.0,
            angular_damping: 0.0,
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
            rest_length,
            ..self
        }
    }

    pub fn set_compliance(self, compliance: f32) -> Self {
        Self { compliance, ..self }
    }

    pub fn set_lagrange_multiplier(self, lagrange_multiplier: f32) -> Self {
        Self {
            lagrange_multiplier,
            ..self
        }
    }

    pub fn set_force(self, force: Vec3) -> Self {
        Self { force, ..self }
    }

    pub fn set_velocity_damping(self, velocity_damping: f32) -> Self {
        Self {
            velocity_damping,
            ..self
        }
    }

    pub fn set_angular_damping(self, angular_damping: f32) -> Self {
        Self {
            angular_damping,
            ..self
        }
    }

    fn solve_constraint(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32) -> Vec3 {
        let [body1, body2] = rigid_bodys;
        let world_r1 = body1.curr_transform.rotation * self.local_anchor1;
        let world_r2 = body2.curr_transform.rotation * self.local_anchor2;

        let p1 = body1.get_world_position() + world_r1;
        let p2 = body2.get_world_position() + world_r2;
        let (normal, c) = self.compute_correction_pair(p1, p2);
        if c.abs() <= f32::EPSILON {
            return Vec3::ZERO;
        }

        let body1_w =
            XPBDPositionConstraint::compute_generalized_inverse_mass(self, body1, normal, world_r1);
        let body2_w =
            XPBDPositionConstraint::compute_generalized_inverse_mass(self, body2, normal, world_r2);
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

    fn solve_joint_damping(&mut self, rigid_bodys: [&mut RigidBodyQueryItem; 2], dt: f32) {
        let [body1, body2] = rigid_bodys;
        let world_r1 = body1.curr_transform.rotation * self.local_anchor1;
        let world_r2 = body2.curr_transform.rotation * self.local_anchor2;

        // delta_v = (v2 - v1) * min(velocity_damping * dt, 1)
        let v1 = body1.velocity.0;
        let v2 = body2.velocity.0;
        let delta_v = (v2 - v1) * (self.velocity_damping() * dt).min(1.0);

        // delta_omega = (w2 - w1) * min(angular_damping * dt, 1)
        let omega1 = body1.angular_velocity.0;
        let omega2 = body2.angular_velocity.0;
        let delta_omega = (omega2 - omega1) * (self.angular_damping() * dt).min(1.0);
        let w1 = if body1.rigid_type.is_dynamic() {
            body1.get_inv_mass()
        } else {
            0.0
        };
        let w2 = if body2.rigid_type.is_dynamic() {
            body2.get_inv_mass()
        } else {
            0.0
        };
        let w_sum = w1 + w2;
        if w_sum.abs() <= f32::EPSILON {
            return;
        }

        if body1.rigid_type.is_dynamic() {
            // velocity
            let inv_mass1 = body1.get_inv_mass();
            // let inv_inertia1 = body1.compute_world_inv_interia();
            let impluse = delta_v / w_sum;
            body1.velocity.0 += impluse * inv_mass1;
            //  angular velocity
            // impluse = delta_omega / w_sum;
            // body1.angular_velocity.0 += inv_inertia1 * world_r1.cross(impluse);
            body1.angular_velocity.0 += delta_omega;
        }
        if body2.rigid_type.is_dynamic() {
            // velocity
            let inv_mass2: f32 = body2.get_inv_mass();
            // let inv_inertia2 = body2.compute_world_inv_interia();
            let impluse = delta_v / w_sum;
            body2.velocity.0 -= impluse * inv_mass2;
            //  angular velocity
            // impluse = delta_omega / w_sum;
            // body2.angular_velocity.0 -= inv_inertia2 * world_r2.cross(impluse);
            body1.angular_velocity.0 -= delta_omega;
        }
    }

    // compute distance joint correction force
    fn compute_correction_pair(&self, pos1: Vec3, pos2: Vec3) -> (Vec3, f32) {
        let delta_x = pos2 - pos1;
        let distance = delta_x.length();
        if distance <= f32::EPSILON {
            return (Vec3::ZERO, 0.0);
        }
        (-delta_x / distance, distance - self.rest_length)
    }
}
