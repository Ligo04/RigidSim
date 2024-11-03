use bevy::math::VectorSpace;
use bevy::prelude::*;
use RigidSim::physics::compoment::*;
use RigidSim::physics::rigidbody::RigidBodyBundle;
use RigidSim::plugins::controller::{CameraController, CameraControllerPlugin};
use RigidSim::plugins::fps_show::FrameShowPlugin;
use RigidSim::solver::{distance_joint::DistanceJoint, XpbdSolverPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(FrameShowPlugin)
        .add_plugins(CameraControllerPlugin)
        .add_plugins(XpbdSolverPlugin)
        .insert_resource(ClearColor(Color::srgb(0.0, 0.0, 0.0)))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cuboid_size = Vec3::new(1.0, 1.0, 1.0);
    let cuboid_material = materials.add(Color::srgb(0.8, 0.7, 0.6));

    let chain_distance: f32 = 1.5;
    let chain_count: i32 = 10;
    let rest_length: f32 = chain_distance;

    let init_pos_y: f32 = 2.0;

    // camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 0.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
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
        .spawn(RigidBodyBundle::new_cuboid(
            &mut meshes,
            cuboid_material.clone(),
            Transform::from_xyz(0.0, init_pos_y, 0.0),
            RigidBodyType::Static,
            cuboid_size,
            1.0,
        ))
        .id();

    let mut cubioc1 = commands
        .spawn(RigidBodyBundle::new_cuboid(
            &mut meshes,
            cuboid_material.clone(),
            Transform::from_xyz(2.0, init_pos_y - 1.5, 0.0),
            RigidBodyType::Dynamic,
            cuboid_size,
            1.0,
        ))
        .id();

    commands.spawn(
        DistanceJoint::new(cubioc0, cubioc1)
            .set_anchor2(0.5 * Vec3::ONE)
            .set_rest_length(rest_length)
            .set_compliance(0.01),
    );
    for i in 1..chain_count {
        cubioc0 = cubioc1;
        cubioc1 = commands
            .spawn(RigidBodyBundle::new_cuboid(
                &mut meshes,
                cuboid_material.clone(),
                Transform::from_xyz(2.0, init_pos_y - 1.5 * (i + 1) as f32, 0.0),
                RigidBodyType::Dynamic,
                cuboid_size,
                1.0,
            ))
            .id();
        println!("cubioc0:{:?}, cubioc1:{:?}", cubioc0, cubioc1);
        commands.spawn(
            DistanceJoint::new(cubioc0, cubioc1)
                .set_anchor1(-0.5 * Vec3::ONE)
                .set_anchor2(0.5 * Vec3::ONE)
                .set_rest_length(rest_length)
                .set_compliance(0.01),
        );
    }
}
