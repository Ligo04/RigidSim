use std::f32::consts::PI;

use bevy::{input::mouse::MouseMotion, prelude::*};
pub const RADIANS_PER_DOT: f32 = 1.0 / 180.0;

// Create Camera Controller Plugin.
pub struct CameraControllerPlugin;

impl Plugin for CameraControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, update_camera_controller);
    }
}

// Create Camera Controller Component.
#[derive(Component, Debug)]
pub struct CameraController {
    pub enabled: bool,
    pub initialized: bool,
    pub sensitivity: f32,
    pub key_forward: KeyCode,
    pub key_back: KeyCode,
    pub key_left: KeyCode,
    pub key_right: KeyCode,
    pub key_up: KeyCode,
    pub key_down: KeyCode,
    pub yaw: f32,
    pub pitch: f32,
    pub velocity: Vec3,
    pub friction: f32,
    pub mouse_key_cursor_grab: MouseButton,
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            enabled: true,
            initialized: false,
            sensitivity: 1.0,
            key_forward: KeyCode::KeyW,
            key_back: KeyCode::KeyS,
            key_left: KeyCode::KeyA,
            key_right: KeyCode::KeyD,
            key_up: KeyCode::Space,
            key_down: KeyCode::ShiftLeft,
            yaw: 0.0,
            pitch: 0.0,
            velocity: Vec3::ZERO,
            friction: 0.15,
            mouse_key_cursor_grab: MouseButton::Right,
        }
    }
}

fn update_camera_controller(
    time: Res<Time>,
    key_input: Res<ButtonInput<KeyCode>>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut mouse_events: EventReader<MouseMotion>,
    mut query: Query<(&mut Transform, &mut CameraController), With<Camera>>,
) {
    let delta_time = time.delta_seconds();

    if let Ok((mut transform, mut controller)) = query.get_single_mut() {
        if !controller.enabled {
            return;
        }
        if !controller.initialized {
            let (yaw, pitch, _roll) = transform.rotation.to_euler(EulerRot::YXZ);
            controller.yaw = yaw;
            controller.pitch = pitch;
            controller.initialized = true;
        }

        // Handle Key input.
        let mut axis_input = Vec3::ZERO;
        if key_input.pressed(controller.key_forward) {
            axis_input.z += 1.0;
        }
        if key_input.pressed(controller.key_back) {
            axis_input.z -= 1.0;
        }
        if key_input.pressed(controller.key_right) {
            axis_input.x += 1.0;
        }
        if key_input.pressed(controller.key_left) {
            axis_input.x -= 1.0;
        }
        if key_input.pressed(controller.key_up) {
            axis_input.y += 1.0;
        }
        if key_input.pressed(controller.key_down) {
            axis_input.y -= 1.0;
        }

        // Apply movement update.
        if axis_input != Vec3::ZERO {
            let max_speed = 5.0;
            controller.velocity = axis_input.normalize() * max_speed;
        } else {
            let friction = controller.friction.clamp(0.0, 1.0);
            controller.velocity *= 1.0 - friction;
            if controller.velocity.length_squared() < 1e-6 {
                controller.velocity = Vec3::ZERO;
            }
        }
        let forward = (*transform.forward() * Vec3::new(1.0, 0.0, 1.0)).normalize();
        let right = *transform.right();
        transform.translation += controller.velocity.x * delta_time * right
            + controller.velocity.y * delta_time * Vec3::Y
            + controller.velocity.z * delta_time * forward;

        // Handle Mouse Input
        let mut mouse_delta = Vec2::ZERO;
        if mouse_button.pressed(controller.mouse_key_cursor_grab) {
            for mouse_event in mouse_events.read() {
                mouse_delta += mouse_event.delta;
            }
        } else {
            mouse_events.clear();
        }
        if mouse_delta != Vec2::ZERO {
            controller.pitch = (controller.pitch
                - mouse_delta.y * RADIANS_PER_DOT * controller.sensitivity)
                .clamp(-PI / 2., PI / 2.);
            controller.yaw -= mouse_delta.x * RADIANS_PER_DOT * controller.sensitivity;
            transform.rotation =
                Quat::from_euler(EulerRot::ZYX, 0.0, controller.yaw, controller.pitch);
        }
    }
}
