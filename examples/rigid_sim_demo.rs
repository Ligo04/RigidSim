use bevy::prelude::*;
use RigidSim::plugins::controller::{CameraController, CameraControllerPlugin};
use RigidSim::plugins::fps_show::FrameShowPlugin;
use RigidSim::solver::XpbdSolverPlugin;

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
    // camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 0.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
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

    let cube_mesh = meshes.add(Cuboid::default());
    let cube_material = materials.add(Color::srgb(0.8, 0.7, 0.6));

    // Spawn a static cube and a dynamic cube that is connected to it by a distance joint.
    let static_cube = commands
        .spawn((PbrBundle {
            mesh: cube_mesh.clone(),
            material: cube_material.clone(),
            transform: Transform::from_xyz(0.0, 2.0, 0.0),
            ..default()
        },))
        .id();
    let dynamic_cube = commands
        .spawn((PbrBundle {
            mesh: cube_mesh,
            material: cube_material,
            transform: Transform::from_xyz(-2.0, 1.5, 0.0),
            ..default()
        },))
        .id();

    //TODO:add distance joint
}
