use bevy::{ecs::query::QueryData, prelude::*};

const GRAVITY: Vec3 = Vec3::new(0.0, -9.8, 0.0);

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

#[derive(Component)]
pub struct Mass {
    inv_mass: f32,
}

impl Default for Mass {
    fn default() -> Self {
        Self { inv_mass: 0.0 }
    }
}

impl Mass {
    pub const INFINITY: Mass = Mass { inv_mass: 0.0 };
}
#[derive(Component)]
pub struct Inertia {
    inv_inertia: Mat3,
}

impl Default for Inertia {
    fn default() -> Self {
        Self {
            inv_inertia: Mat3::IDENTITY,
        }
    }
}

impl Inertia {
    pub const INFINITY: Inertia = Inertia {
        inv_inertia: Mat3::ZERO,
    };
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct RigidBody {
    entity: Entity,
    rigid_type: Ref<'static, RigidBodyType>,
    mass: &'static mut Mass,
    center_of_mass: &'static mut Mass,
    inertia: &'static mut Inertia,
    velocity: &'static mut Mass,
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
            self.mass.inv_mass + rn.dot(interia.inv_inertia * rn)
        } else {
            0.0
        }
    }
}
