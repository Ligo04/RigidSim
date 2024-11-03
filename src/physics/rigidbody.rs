use bevy::{ecs::query::QueryData, prelude::*};

use super::compoment::*;

#[derive(Bundle, Default)]
pub struct RigidBodyBundle {
    pub pbr_bundle: PbrBundle,
    pub rigid_type: RigidBodyType,
    pub mass: Mass,
    pub inertia: Inertia,
    pub center_of_mass: CentorOfMass,
    pub velocity: Velocity,
    pub angular_velocity: AngularVelocity,
    pub prev_transform: PrevTransform,
}

impl RigidBodyBundle {
    pub fn new_from_cuboid(
        mesh: &mut ResMut<Assets<Mesh>>,
        material: Handle<StandardMaterial>,
        transform: Transform,
        rigid_type: RigidBodyType,
        size: Vec3,
        density: f32,
    ) -> Self {
        Self {
            pbr_bundle: PbrBundle {
                mesh: mesh.add(Cuboid::new(size.x, size.y, size.z)),
                material,
                transform,
                ..Default::default()
            },
            rigid_type,
            mass: Mass::from_cubiod(size, density),
            inertia: Inertia::from_cubiod(size, density),
            ..Default::default()
        }
    }
}

#[derive(QueryData, Debug)]
#[query_data(mutable)]
pub struct RigidBodyQuery {
    pub entity: Entity,
    pub rigid_type: Ref<'static, RigidBodyType>,
    pub mass: &'static mut Mass,
    pub center_of_mass: &'static mut CentorOfMass,
    pub inertia: &'static mut Inertia,
    pub velocity: &'static mut Velocity,
    pub angular_velocity: &'static mut AngularVelocity,
    pub prev_transform: &'static mut PrevTransform,
    pub curr_transform: &'static mut Transform,
}

impl<'w> RigidBodyQueryItem<'w> {
    pub fn compute_world_inv_interia(&self) -> Mat3 {
        if !self.rigid_type.is_dynamic() {
            Inertia::INFINITY.inverse()
        } else {
            self.inertia.inverse()
        }
    }

    pub fn get_inv_mass(&self) -> f32 {
        if !self.rigid_type.is_dynamic() {
            0.0
        } else {
            self.mass.inverse()
        }
    }

    // TODO: implement this
    pub fn get_world_position(&self) -> Vec3 {
        self.curr_transform.translation
    }
}
