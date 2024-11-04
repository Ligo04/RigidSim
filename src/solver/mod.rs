pub mod distance_joint;
pub mod xpbd_constraint;

use bevy::prelude::*;
use distance_joint::{DistanceJoint, Joint};
use xpbd_constraint::XPBDConstraint;

use crate::physics::rigidbody::{RigidBodyQuery, RigidBodyQueryItem};

const GRAVITY: Vec3 = Vec3::new(0.0, -9.8, 0.0);
fn skew(v: Vec3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new(0.0, v.z, -v.y),
        Vec3::new(-v.z, 0.0, v.x),
        Vec3::new(v.y, -v.x, 0.0),
    )
}
#[derive(Resource, Clone, Copy)]
pub struct SubStepCount(pub u32);

impl Default for SubStepCount {
    fn default() -> Self {
        Self(5)
    }
}

pub struct XpbdSolverPlugin;

impl Plugin for XpbdSolverPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(SubStepCount(6))
            .add_systems(FixedUpdate, XpbdSolverPlugin::substep::<DistanceJoint>)
            .add_systems(FixedPostUpdate, line_draw::<DistanceJoint>);
    }
}

impl XpbdSolverPlugin {
    pub fn substep<C: XPBDConstraint + Component + Joint>(
        time: Res<Time>,
        sub_step_count: Res<SubStepCount>,
        mut bodies: Query<RigidBodyQuery>,
        mut constraints: Query<&mut C>,
    ) {
        let dt = time.delta_seconds() / sub_step_count.0 as f32;

        if dt.abs() <= f32::EPSILON {
            return;
        }
        for _i in 0..sub_step_count.0 {
            Self.presolve(&mut bodies, dt as f32);
            Self.solve_constraint(&mut bodies, &mut constraints, dt as f32);
            Self.update_velocity(&mut bodies, dt as f32);
            Self.solve_velocity(&mut bodies, &mut constraints, dt);
        }
    }

    fn solve_constraint<C: XPBDConstraint + Component>(
        &self,
        bodies: &mut Query<RigidBodyQuery>,
        constraint: &mut Query<&mut C>,
        dt: f32,
    ) {
        // clear lagrange multiplier
        constraint
            .iter_mut()
            .for_each(|mut c| c.clear_lagrange_multiplier());

        for mut constraint in constraint.iter_mut() {
            // get the constraint rigid bodies
            if let Ok(mut constraint_bodies) = bodies.get_many_mut(constraint.entities()) {
                if let Ok(rigid_bodies) = constraint_bodies
                    .iter_mut()
                    .collect::<Vec<&mut RigidBodyQueryItem>>()
                    .try_into()
                {
                    constraint.solve(rigid_bodies, dt as f32);
                }
            }
        }
    }

    fn presolve(&self, bodies: &mut Query<RigidBodyQuery>, dt: f32) {
        let dynamic_bodies = bodies.iter_mut().filter(|b| b.rigid_type.is_dynamic());
        for mut body in dynamic_bodies {
            let force = body.externel_force.force;
            let inv_mass = body.get_inv_mass();
            // xprev = x
            body.prev_transform.0.translation = body.curr_transform.translation;
            // v = v + dt * f_ext/m (G)
            let linear_acceleration = force * inv_mass + GRAVITY;
            body.velocity.0 += dt * linear_acceleration;
            // semi-implicit
            body.curr_transform.translation += dt * body.velocity.0;
            // qprev = q
            body.prev_transform.0.rotation = body.curr_transform.rotation;
            let inv_inertia = body.compute_world_inv_interia();
            let torque: Vec3 = body.externel_force.torque;
            let mut angular_acceleration = inv_inertia * torque;
            let angular_vec = body.angular_velocity.0;
            // semi-implicit : w = w + dt * I^-1 * (t_ext - (w x (Iw)))
            // angular_acceleration -=
            //     inv_inertia * angular_vec.cross(inv_inertia.inverse() * angular_vec);
            // gryosopic:  w = w + dt * I^-1 * t_ext + (quadratic in omega)
            // https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf
            angular_acceleration += Self::solve_gryosopic(
                body.curr_transform.rotation,
                inv_inertia.inverse(),
                angular_vec,
                dt,
            );
            body.angular_velocity.0 += dt * angular_acceleration;
            // detla_q =  dt * 0.5 * [w,0] * q
            let omega = 0.5 * dt * body.angular_velocity.0;
            let delta_q = Quat::from_xyzw(omega.x, omega.x, omega.z, 0.0)
                .mul_quat(body.curr_transform.rotation);
            body.curr_transform.rotation = body.curr_transform.rotation + delta_q;
            body.curr_transform.rotation = body.curr_transform.rotation.normalize();
            // clear external force
            body.externel_force.clear();
        }
    }

    fn update_velocity(&self, bodies: &mut Query<RigidBodyQuery>, dt: f32) {
        Self.project_linear_velocity(bodies, dt);
        Self.project_angular_velocity(bodies, dt);
    }

    fn solve_velocity<C: XPBDConstraint + Component + Joint>(
        &self,
        bodies: &mut Query<RigidBodyQuery>,
        constraint: &mut Query<&mut C>,
        dt: f32,
    ) {
        self.solve_joint_damping(bodies, constraint, dt);
    }

    fn solve_joint_damping<C: XPBDConstraint + Component + Joint>(
        &self,
        bodies: &mut Query<RigidBodyQuery>,
        constraint: &mut Query<&mut C>,
        dt: f32,
    ) {
        for mut constraint in constraint.iter_mut() {
            // get the constraint rigid bodies
            if let Ok(mut constraint_bodies) = bodies.get_many_mut(constraint.entities()) {
                if let Ok(rigid_bodies) = constraint_bodies
                    .iter_mut()
                    .collect::<Vec<&mut RigidBodyQueryItem>>()
                    .try_into()
                {
                    constraint.solve_joint_damping(rigid_bodies, dt);
                }
            }
        }
    }

    fn project_linear_velocity(&self, bodies: &mut Query<RigidBodyQuery>, dt: f32) {
        for mut body in bodies.iter_mut() {
            if body.rigid_type.is_dynamic() {
                // v = (x - x_prev) / h
                let delta_x = body.curr_transform.translation - body.prev_transform.0.translation;
                body.velocity.0 = delta_x / dt;
            } else {
                body.velocity.0 = Vec3::ZERO;
            }
        }
    }

    fn project_angular_velocity(&self, bodies: &mut Query<RigidBodyQuery>, dt: f32) {
        // delta_q = q * q_prev^-1
        // w = 2[delta_q_x,delta_q_y,delta_q_z] / h
        // w = delta_q_w>0 ? w : -w
        for mut body in bodies.iter_mut() {
            let q = body.curr_transform.rotation;
            let q_prev = body.prev_transform.0.rotation;
            let delta_q = q.mul_quat(q_prev.inverse());
            let delta_w = 2.0 * delta_q.xyz() / dt;
            if delta_q.w >= 0.0 {
                body.angular_velocity.0 = delta_w;
            } else {
                body.angular_velocity.0 = -delta_w;
            }
        }
    }

    fn solve_gryosopic(rotation: Quat, inertia: Mat3, angular_vec: Vec3, dt: f32) -> Vec3 {
        let local_omgea = rotation.inverse() * angular_vec;

        let f = dt * local_omgea.cross(inertia * local_omgea);

        let jacobian = inertia + dt * (skew(local_omgea) * inertia - skew(inertia * local_omgea));

        let delta_omega = -jacobian.inverse() * f;

        let result = rotation * delta_omega;
        result
    }
}

fn line_draw<C: XPBDConstraint + Component + Joint>(
    mut gizmos: Gizmos,
    bodies: Query<RigidBodyQuery>,
    constaints: Query<&mut C>,
) {
    for constaint in constaints.iter() {
        // get the constraint rigid bodies
        if let Ok(bodies) = bodies.get_many(constaint.entities()) {
            let point1 = bodies[0].curr_transform.translation
                + bodies[0].curr_transform.rotation * constaint.local_anchor1();
            let point2 = bodies[1].curr_transform.translation
                + bodies[1].curr_transform.rotation * constaint.local_anchor2();

            gizmos.line(point1, point2, LinearRgba::RED);
        }
    }
}
