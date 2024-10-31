use bevy::{ecs::query::QueryData, prelude::*};

use super::math::*;

#[derive(Component)]
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

#[derive(Component, QueryData)]
#[query_data(mutable)]
pub struct RigidBody {
    entity: Entity,
    rigid_type: Ref<'static, RigidBodyType>,
    mass: &'static mut Mass,
    center_of_mass: &'static mut CentorOfMass,
    inertia: &'static mut Inertia,
    velocity: &'static mut Velocity,
    angular_velocity: &'static mut Inertia,
    prev_transform: &'static mut Transform,
}

impl RigidBody {
    fn compute_world_inv_interia(&self) -> Inertia {
        if !self.rigid_type.is_dynamic() {
            Inertia::INFINITY
        } else {
            Inertia::INFINITY
        }
    }

    fn compute_generalized_inverse_mass(&self, normal: Vec3, r: Vec3) -> f32 {
        if self.rigid_type.is_dynamic() {
            let rn = r.cross(normal);
            let interia = self.compute_world_inv_interia();
            self.mass.inverse() + rn.dot(interia.inverse() * rn)
        } else {
            0.0
        }
    }
}
