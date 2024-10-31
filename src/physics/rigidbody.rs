use bevy::{ecs::query::QueryData, prelude::*};

use super::math::*;

#[derive(Component, Clone, Copy)]
pub enum RigidBodyType {
    Static,
    Dynamic,
    Kinematic, // not implemented
}

impl RigidBodyType {
    pub fn is_static(&self) -> bool {
        match self {
            RigidBodyType::Static => true,
            _ => false,
        }
    }

    pub fn is_dynamic(&self) -> bool {
        match self {
            RigidBodyType::Dynamic => true,
            _ => false,
        }
    }

    pub fn is_kinematic(&self) -> bool {
        match self {
            RigidBodyType::Kinematic => true,
            _ => false,
        }
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct RigidBody {
    pub entity: Entity,
    pub rigid_type: Ref<'static, RigidBodyType>,
    pub mass: &'static mut Mass,
    pub center_of_mass: &'static mut CentorOfMass,
    pub inertia: &'static mut Inertia,
    pub velocity: &'static mut Velocity,
    pub angular_velocity: &'static mut Velocity,
    pub prev_transform: &'static mut Transform,
    pub accu_transform: &'static mut Transform,
}

impl<'w> RigidBodyItem<'w> {
    pub fn compute_world_inv_interia(&self) -> Inertia {
        if !self.rigid_type.is_dynamic() {
            Inertia::INFINITY
        } else {
            *self.inertia
        }
    }

    pub fn get_inv_mass(&self) -> f32 {
        if !self.rigid_type.is_dynamic() {
            0.0
        } else {
            self.mass.inverse()
        }
    }

    pub fn compute_generalized_inverse_mass(&self, normal: Vec3, r: Vec3) -> f32 {
        if self.rigid_type.is_dynamic() {
            let rn = r.cross(normal);
            let interia = self.compute_world_inv_interia();
            self.mass.inverse() + rn.dot(interia.inverse() * rn)
        } else {
            0.0
        }
    }
}
