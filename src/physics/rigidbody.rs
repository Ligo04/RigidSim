use bevy::prelude::*;

pub enum RigidBodyItem {
    Static,
    Dynamic,
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

#[derive(Component)]
pub struct RigidBody {
    entity: Entity,
    mass: Mass,
    inertia: Inertia,
    velocity: Vec3,
    angular_velocity: Vec3,
    prev_transform: Transform,
}
