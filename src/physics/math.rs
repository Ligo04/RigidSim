use bevy::prelude::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.8, 0.0);
#[derive(Component, Clone, Copy)]
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
    pub fn inverse(&self) -> f32 {
        self.inv_mass
    }
}
#[derive(Component, Clone, Copy)]
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

    pub fn inverse(&self) -> Mat3 {
        self.inv_inertia
    }
}

#[derive(Component, Clone, Copy, Default)]
pub struct CentorOfMass(pub Vec3);

#[derive(Component, Clone, Copy, Default)]
pub struct Velocity(pub Vec3);

#[derive(Component, Clone, Copy, Default)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Clone, Copy, Default)]
pub struct CurrTransform(pub Transform);
#[derive(Component, Clone, Copy, Default)]
pub struct PrevTransform(pub Transform);
