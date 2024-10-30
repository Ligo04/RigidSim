use bevy::animation::transition;

pub trait XPBDConstraint {
    fn solve(&self, dt: f32);
    fn clear_lagrange_multiplier(&mut self);
}

pub trait XPBDPositionConstraint: XPBDConstraint {}

pub trait XPBDAngularConstraint: XPBDConstraint {}
