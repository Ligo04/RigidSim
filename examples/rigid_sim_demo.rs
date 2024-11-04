use bevy::prelude::*;
use bevy_mod_picking::prelude::*;
use std::env;
// use bevy::window::Windows;
use rigid_sim::physics::compoment::*;
use rigid_sim::physics::rigidbody::{RigidBodyBundle, RigidBodyQuery};
use rigid_sim::plugins::controller::{CameraController, CameraControllerPlugin};
use rigid_sim::plugins::fps_show::FrameShowPlugin;
use rigid_sim::solver::{distance_joint::DistanceJoint, XpbdSolverPlugin};
#[derive(Resource, Clone, Copy)]
struct ChainCount(i32);

#[derive(Resource, Clone, Copy)]
struct RestDistance(f32);

#[derive(Resource, Clone, Copy)]
struct CuboidSize(Vec3);

#[derive(Resource, Clone, Copy)]
struct Compliance(f32);

#[derive(Resource, Clone, Copy)]
struct LinearVelDamping(f32);

#[derive(Resource, Clone, Copy)]
struct AngularVelDamping(f32);

fn main() {
    let args: Vec<String> = env::args().collect();
    let size_x: f32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let size_y: f32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let size_z: f32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(1.0);

    let chain_count: i32 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(1);
    let distance: f32 = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(1.5);

    let compliance: f32 = args.get(6).and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let linear_vel_damping: f32 = args.get(7).and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let angular_vel_damping: f32 = args.get(8).and_then(|s| s.parse().ok()).unwrap_or(0.0);

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(FrameShowPlugin)
        .add_plugins(CameraControllerPlugin)
        .add_plugins(DefaultPickingPlugins)
        .add_plugins(XpbdSolverPlugin)
        .insert_resource(ClearColor(Color::srgb(0.0, 0.0, 0.0)))
        .insert_resource(ChainCount(chain_count))
        .insert_resource(RestDistance(distance))
        .insert_resource(CuboidSize(Vec3::new(size_x, size_y, size_z)))
        .insert_resource(Compliance(compliance))
        .insert_resource(LinearVelDamping(linear_vel_damping))
        .insert_resource(AngularVelDamping(angular_vel_damping))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    chain_count: Res<ChainCount>,
    rest_distance: Res<RestDistance>,
    cuboid_size: Res<CuboidSize>,
    compliacne: Res<Compliance>,
    linear_vel_damping: Res<LinearVelDamping>,
    angular_vel_damping: Res<AngularVelDamping>,
) {
    let cuboid_material = materials.add(Color::srgb(0.8, 0.7, 0.6));
    let cuboid_size = cuboid_size.0;

    let chain_distance: f32 = rest_distance.0;
    let chain_count: i32 = chain_count.0;
    let rest_length: f32 = chain_distance;

    let init_pos_y: f32 = 2.0;

    // camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 0.0, 9.0 + 1.0 * chain_count as f32)
                .looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        CameraController::default(),
    ));

    // Light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::default().looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Z),
        ..default()
    });
    // Spawn a static cube and a dynamic cube that is connected to it by a distance joint.
    let mut cubioc0 = commands
        .spawn((
            RigidBodyBundle::new_from_cuboid(
                &mut meshes,
                cuboid_material.clone(),
                Transform::from_xyz(0.0, init_pos_y, 0.0),
                RigidBodyType::Static,
                Vec3::new(0.1, 0.1, 0.1),
                1.0,
            ),
            PickableBundle::default(),
        ))
        .id();

    let mut cubioc1 = commands
        .spawn((
            RigidBodyBundle::new_from_cuboid(
                &mut meshes,
                cuboid_material.clone(),
                Transform::from_xyz(2.0, init_pos_y - 1.5, 0.0),
                RigidBodyType::Dynamic,
                cuboid_size,
                1.0,
            ),
            PickableBundle::default(),
            On::<Pointer<Click>>::run(add_external_force),
        ))
        .id();

    commands.spawn(
        DistanceJoint::new(cubioc0, cubioc1)
            .set_anchor2(0.5 * cuboid_size)
            .set_rest_length(rest_length)
            .set_compliance(compliacne.0)
            .set_velocity_damping(linear_vel_damping.0)
            .set_angular_damping(angular_vel_damping.0),
    );
    for i in 1..chain_count {
        cubioc0 = cubioc1;
        cubioc1 = commands
            .spawn((
                RigidBodyBundle::new_from_cuboid(
                    &mut meshes,
                    cuboid_material.clone(),
                    Transform::from_xyz(2.0, init_pos_y - 1.5 * (i + 1) as f32, 0.0),
                    RigidBodyType::Dynamic,
                    cuboid_size,
                    1.0,
                ),
                PickableBundle::default(),
                On::<Pointer<Click>>::run(add_external_force),
            ))
            .id();
        commands.spawn(
            DistanceJoint::new(cubioc0, cubioc1)
                .set_anchor1(-0.5 * cuboid_size)
                .set_anchor2(0.5 * cuboid_size)
                .set_rest_length(rest_length)
                .set_compliance(compliacne.0)
                .set_velocity_damping(linear_vel_damping.0)
                .set_angular_damping(angular_vel_damping.0),
        );
    }
}

fn add_external_force(click: Listener<Pointer<Click>>, mut bodies: Query<RigidBodyQuery>) {
    let hitdata = click.hit.clone();
    println!("hitdata: {:?}", hitdata);
    if let Ok(mut click_body) = bodies.get_mut(click.target) {
        println!("click_body: {:?}", click_body.entity);
        let positon = hitdata.position.unwrap();
        let centor_of_mass = click_body.center_of_mass.0 + click_body.get_world_position();
        let force: Vec3 = -100.0 * Vec3::Z;
        let external_force = ExternelForce::new_from_point(force, positon, centor_of_mass, false);
        *click_body.externel_force = external_force;
        println!("click_body.externel_force: {:?}", click_body.externel_force);
    }
}
