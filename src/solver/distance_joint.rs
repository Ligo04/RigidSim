use bevy::prelude::*;

#[derive(Component)]
pub struct DistanceJoint {
    pub entity1: Entity,
    pub entity2: Entity,
    pub distance: f32,
    pub stiffness: f32,
    pub damping: f32,
}


