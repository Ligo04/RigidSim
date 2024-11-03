use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug)]
pub enum RigidBodyType {
    Static,
    Dynamic,
    Kinematic, // not implemented
}

impl Default for RigidBodyType {
    fn default() -> Self {
        RigidBodyType::Dynamic
    }
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

#[derive(Component, Clone, Copy, Debug)]
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
    pub fn new(mass: f32) -> Self {
        Self {
            inv_mass: 1.0 / mass,
        }
    }

    pub fn from_cubiod(size: Vec3, density: f32) -> Self {
        let volume = size.x * size.y * size.z;
        let mass = density * volume;
        Self {
            inv_mass: 1.0 / mass,
        }
    }

    pub fn inverse(&self) -> f32 {
        self.inv_mass
    }
}
#[derive(Component, Clone, Copy, Debug)]
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

    pub fn new(inertia: Mat3) -> Self {
        if inertia.determinant() == 0.0 {
            Self {
                inv_inertia: Mat3::ZERO,
            }
        } else {
            Self {
                inv_inertia: inertia.inverse(),
            }
        }
    }

    pub fn from_cubiod(size: Vec3, mass: f32) -> Self {
        Self {
            inv_inertia: Mat3::from_diagonal(Vec3::new(
                1.0 / 12.0 * mass * (size.y * size.y + size.z * size.z), // 1/12 * m * (y^2 + z^2)
                1.0 / 12.0 * mass * (size.x * size.x + size.z * size.z), // 1/12 * m * (x^2 + z^2)
                1.0 / 12.0 * mass * (size.x * size.x + size.y * size.y), // 1/12 * m * (x^2 + y^2)
            )),
        }
    }

    pub fn inverse(&self) -> Mat3 {
        self.inv_inertia
    }
}

#[derive(Component, Clone, Copy, Default, Debug)]
pub struct CentorOfMass(pub Vec3);

#[derive(Component, Clone, Copy, Default, Debug)]
pub struct Velocity(pub Vec3);

#[derive(Component, Clone, Copy, Default, Debug)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Clone, Copy, Default, Debug)]
pub struct CurrTransform(pub Transform);
#[derive(Component, Clone, Copy, Default, Debug)]
pub struct PrevTransform(pub Transform);
