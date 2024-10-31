use bevy::{
    ecs::entity::{Entities, MapEntities},
    prelude::Entity,
};

use crate::physics::rigidbody::RigidBodyItem;

pub trait XPBDConstraint: MapEntities {
    fn entities(&self) -> [&Entity; 2];
    fn clear_lagrange_multiplier(&mut self);
    fn solve(&mut self, rigid_bodys: [&mut RigidBodyItem; 2], dt: f32);
}

pub trait XPBDPositionConstraint: XPBDConstraint {}

pub trait XPBDAngularConstraint: XPBDConstraint {}
