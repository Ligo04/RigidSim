pub mod distance_joint;
pub mod xpbd_constraint;

use bevy::prelude::*;
use distance_joint::{DistanceJoint, Joint};
use xpbd_constraint::XPBDConstraint;

use crate::physics::rigidbody::{RigidBodyQuery, RigidBodyQueryItem};
const GRAVITY: Vec3 = Vec3::new(0.0, -9.8, 0.0);

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
            .add_systems(Update, XpbdSolverPlugin::substep::<DistanceJoint>)
            .add_systems(PostUpdate, debug_draw::<DistanceJoint>);
    }
}

impl XpbdSolverPlugin {
    pub fn substep<C: XPBDConstraint + Component + Joint>(
        time: Res<Time>,
        sub_step_count: Res<SubStepCount>,
        mut bodies: Query<RigidBodyQuery>,
        mut constraint: Query<&mut C>,
    ) {
        let dt = time.delta_seconds() / sub_step_count.0 as f32;

        if dt <= f32::EPSILON {
            return;
        }
        for _i in 0..sub_step_count.0 {
            Self.presolve(&mut bodies, dt as f32);
            Self.solve_constraint(&mut bodies, &mut constraint, dt as f32);
            Self.update_velocity(&mut bodies, dt as f32);
            Self.solve_velocity(&mut bodies, &mut constraint, dt);
        }

        let dynamic_bodies = bodies.iter().filter(|b| b.rigid_type.is_dynamic());
        for body in dynamic_bodies {
            println!("veclocity: {:?}", body.velocity.0);
            println!("angular velocity: {:?}", body.angular_velocity.0);
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
            // xprev = x
            body.prev_transform.0 = *body.curr_transform;
            // v = v + dt * f_ext/m (G)
            body.velocity.0 += dt * GRAVITY;
            // body.curr_transform.translation
            body.curr_transform.translation += dt * body.velocity.0;

            // w = w + dt * I^-1 * (t_ext - (w x (Iw)))
            // body.angular_velocity.0 += dt * body.inertia.inverse().inverse()
            // detla_q =  dt * 0.5 * [w,0] * q
            let delta_w = dt
                * body.inertia.inverse()
                * (-body
                    .angular_velocity
                    .0
                    .cross(body.inertia.inverse().inverse() * body.angular_velocity.0));
            body.angular_velocity.0 += delta_w;
            let omega = body.angular_velocity.0;
            let mut delta_q = Quat::from_xyzw(omega.x, omega.x, omega.z, 0.0)
                .mul_quat(body.curr_transform.rotation);
            delta_q = Quat::from_xyzw(
                dt * 0.5 * delta_q.x,
                dt * 0.5 * delta_q.y,
                dt * 0.5 * delta_q.z,
                dt * 0.5 * delta_q.z,
            );
            body.curr_transform.rotation = body.curr_transform.rotation + delta_q;
            body.curr_transform.rotation = body.curr_transform.rotation.normalize();
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
        // TODO: implement contact
        self.solve_joint_damping(bodies, constraint, dt);
    }

    fn solve_joint_damping<C: XPBDConstraint + Component + Joint>(
        &self,
        bodies: &mut Query<RigidBodyQuery>,
        constraint: &mut Query<&mut C>,
        dt: f32,
    ) {
        for constraint in constraint.iter_mut() {
            // get the constraint rigid bodies
            if let Ok(mut constraint_bodies) = bodies.get_many_mut(constraint.entities()) {
                // delta_v = (v2 - v1) * min(velocity_damping * dt, 1)
                let v1 = constraint_bodies[0].velocity.0;
                let v2 = constraint_bodies[1].velocity.0;
                let delta_v = (v2 - v1) * (constraint.velocity_damping() * dt).min(1.0);
                // delta_omega = (omega2 - omega1) * min(angular_velocity_damping * dt, 1)
                let omega1 = constraint_bodies[0].angular_velocity.0;
                let omega2 = constraint_bodies[1].angular_velocity.0;
                let delta_omega = (omega2 - omega1) * (constraint.angular_damping() * dt).min(1.0);

                let w1 = if constraint_bodies[0].rigid_type.is_dynamic() {
                    constraint_bodies[0].mass.inverse()
                } else {
                    0.0
                };
                let w2 = if constraint_bodies[1].rigid_type.is_dynamic() {
                    constraint_bodies[1].mass.inverse()
                } else {
                    0.0
                };
                let w_sum = w1 + w2;
                if w_sum <= f32::EPSILON {
                    continue;
                }

                if constraint_bodies[0].rigid_type.is_dynamic() {
                    // velocity
                    let mut impluse = delta_v / w_sum;
                    constraint_bodies[0].velocity.0 +=
                        impluse * constraint_bodies[0].mass.inverse();
                    //  angular velocity
                    impluse = delta_omega / w_sum;
                    let r1 =
                        constraint_bodies[0].curr_transform.rotation * constraint.local_anchor1();
                    let wolrd_r1 = constraint_bodies[0].curr_transform.translation + r1;
                    constraint_bodies[0].angular_velocity.0 +=
                        constraint_bodies[0].mass.inverse() * wolrd_r1.cross(impluse);
                }
                if constraint_bodies[1].rigid_type.is_dynamic() {
                    // velocity
                    let mut impluse = delta_v / w_sum;
                    constraint_bodies[1].velocity.0 -=
                        impluse * constraint_bodies[1].mass.inverse();
                    //  angular velocity
                    impluse = delta_omega / w_sum;
                    let r2 =
                        constraint_bodies[1].curr_transform.rotation * constraint.local_anchor2();
                    let wolrd_r2 = constraint_bodies[1].curr_transform.translation + r2;
                    constraint_bodies[1].angular_velocity.0 -=
                        constraint_bodies[1].mass.inverse() * wolrd_r2.cross(impluse);
                }
            }
        }
    }

    fn project_linear_velocity(&self, bodies: &mut Query<RigidBodyQuery>, dt: f32) {
        for mut body in bodies.iter_mut() {
            if body.rigid_type.is_dynamic() {
                // v = (x - x_prev) / h
                let delta_vec =
                    (body.curr_transform.translation - body.prev_transform.0.translation) / dt;
                body.velocity.0 = delta_vec;
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
            let delta_q = body
                .curr_transform
                .rotation
                .mul_quat(body.prev_transform.0.rotation.inverse());
            let delta_w = 2.0 * delta_q.xyz() / dt;
            if delta_q.w > 0.0 {
                body.angular_velocity.0 = delta_w;
            } else {
                body.angular_velocity.0 = -delta_w;
            }
        }
    }
}

fn debug_draw<C: XPBDConstraint + Component + Joint>(
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
