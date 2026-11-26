//! Right-drag orbit and wheel zoom input for the Bevy 0.20 pan-orbit controller.
//! Left-click remains available to the organism construction tools.

use bevy::app::prelude::*;
use bevy::camera::{RenderTarget, prelude::*};
use bevy::camera_controller::pan_orbit_camera::prelude::component::PanOrbitCamera;
use bevy::ecs::prelude::*;
use bevy::input::{
    mouse::{MouseScrollUnit, MouseWheel},
    prelude::*,
};
use bevy::math::{DVec2, DVec3, prelude::*};
use bevy::picking::hover::Hovered;
use bevy::picking::pointer::{
    PointerAction, PointerId, PointerInput, PointerInteraction, PointerLocation, PointerMap,
};
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy::window::{PrimaryWindow, Window};

/// The kind of camera motion started by the editor's mouse mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MotionKind {
    Orbit,
    Zoom,
}

/// Input mapping for the pan-orbit controller. The pan mapping is intentionally omitted so that
/// left-click can remain the editor's placement and joint-selection gesture.
pub struct CameraControlsPlugin;

impl Plugin for CameraControlsPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<PanOrbitCameraInputMessage>()
            .init_resource::<CameraPointerMap>()
            .add_systems(
                PreUpdate,
                (
                    custom_camera_inputs,
                    PanOrbitCameraInputMessage::receive_messages,
                    PanOrbitCameraInputMessage::send_pointer_inputs,
                )
                    .chain()
                    .after(bevy::picking::PickingSystems::Last)
                    .before(PanOrbitCamera::update_camera_positions),
            );
    }
}

fn custom_camera_inputs(
    pointers: Query<(&PointerId, &PointerLocation)>,
    pointer_map: Res<CameraPointerMap>,
    mut controller: MessageWriter<PanOrbitCameraInputMessage>,
    mut mouse_wheel: MessageReader<MouseWheel>,
    mouse_input: Res<ButtonInput<MouseButton>>,
    cameras: Query<(Entity, &Camera, &RenderTarget, &PanOrbitCamera)>,
    primary_window: Query<Entity, With<PrimaryWindow>>,
    hovered_panels: Query<&Hovered>,
) {
    let orbit_start = MouseButton::Right;
    let zoom_stop = 0.0;

    if let Some(&camera) = pointer_map.get(&PointerId::Mouse) {
        let camera_query = cameras.get(camera).ok();
        let is_in_zoom_mode = camera_query
            .map(|(.., editor_cam)| editor_cam.current_motion.is_zooming_only())
            .unwrap_or_default();
        let zoom_amount_abs = camera_query
            .and_then(|(.., editor_cam)| {
                editor_cam
                    .current_motion
                    .inputs()
                    .map(|inputs| inputs.zoom_velocity_abs(editor_cam.smoothing.zoom.mul_f32(2.0)))
            })
            .unwrap_or(0.0);
        let should_zoom_end = is_in_zoom_mode && zoom_amount_abs <= zoom_stop;

        if mouse_input.just_released(orbit_start) || should_zoom_end {
            controller.write(PanOrbitCameraInputMessage::End { camera });
        }
    }

    for (&pointer, pointer_location) in pointers
        .iter()
        .filter_map(|(id, location)| location.location().map(|location| (id, location)))
    {
        match pointer {
            PointerId::Mouse => {
                // The root panel's Hovered state includes all descendants.
                if hovered_panels.iter().any(|hovered| hovered.get()) {
                    continue;
                }

                let Some((camera, ..)) = cameras.iter().find(|(_, camera, render_target, _)| {
                    pointer_location.is_in_viewport(camera, render_target, &primary_window)
                }) else {
                    continue;
                };

                if mouse_input.just_pressed(orbit_start) {
                    controller.write(PanOrbitCameraInputMessage::Start {
                        kind: MotionKind::Orbit,
                        camera,
                        pointer,
                    });
                } else if mouse_wheel.read().map(|wheel| wheel.y.abs()).sum::<f32>() > 0.0 {
                    controller.write(PanOrbitCameraInputMessage::Start {
                        kind: MotionKind::Zoom,
                        camera,
                        pointer,
                    });
                }
            }
            PointerId::Touch(_) | PointerId::Custom(_) => continue,
        }
    }

    // Each MessageReader has its own cursor; the input-forwarding system below also reads these
    // wheel messages to forward their actual scroll delta to the camera controller.
    mouse_wheel.clear();
}

/// Maps the active pointer to the camera it is currently controlling.
#[derive(Debug, Clone, Default, Deref, DerefMut, Resource)]
struct CameraPointerMap(HashMap<PointerId, Entity>);

#[derive(Debug, Clone, Message)]
enum PanOrbitCameraInputMessage {
    Start {
        kind: MotionKind,
        camera: Entity,
        pointer: PointerId,
    },
    End {
        camera: Entity,
    },
}

impl PanOrbitCameraInputMessage {
    fn camera(&self) -> Entity {
        match self {
            Self::Start { camera, .. } | Self::End { camera } => *camera,
        }
    }

    fn receive_messages(
        mut events: MessageReader<Self>,
        mut controllers: Query<(&mut PanOrbitCamera, &GlobalTransform)>,
        mut camera_map: ResMut<CameraPointerMap>,
        pointer_map: Res<PointerMap>,
        pointer_interactions: Query<&PointerInteraction>,
        pointer_locations: Query<&PointerLocation>,
        cameras: Query<(&Camera, &Projection)>,
        windows: Query<&Window>,
    ) {
        for event in events.read() {
            let Ok((mut controller, camera_transform)) = controllers.get_mut(event.camera()) else {
                continue;
            };

            match event {
                Self::Start { kind, pointer, .. } => {
                    if controller.is_actively_controlled() {
                        continue;
                    }

                    let anchor = pointer_map
                        .get_entity(*pointer)
                        .and_then(|entity| pointer_interactions.get(entity).ok())
                        .and_then(|interaction| interaction.get_nearest_hit())
                        // A hit on a window is in viewport coordinates, not world coordinates.
                        .filter(|(entity, _hit)| !windows.contains(*entity))
                        .and_then(|(_, hit)| hit.position)
                        .map(|world_position| {
                            camera_transform
                                .to_matrix()
                                .as_dmat4()
                                .inverse()
                                .transform_point3(world_position.into())
                        })
                        .filter(|position| {
                            #[cfg(debug_assertions)]
                            if !position.is_finite() {
                                bevy::log::warn!(
                                    "Non-finite input fed to camera controller: {position:?}"
                                );
                            }
                            position.is_finite()
                        })
                        .or_else(|| {
                            let camera = cameras.get(event.camera()).ok();
                            let pointer_location = pointer_map
                                .get_entity(*pointer)
                                .and_then(|entity| pointer_locations.get(entity).ok())
                                .and_then(|location| location.location());
                            if let Some(((camera, projection), pointer_location)) =
                                camera.zip(pointer_location)
                            {
                                screen_to_view_space(
                                    camera,
                                    projection,
                                    &controller,
                                    pointer_location.position,
                                )
                            } else {
                                None
                            }
                        })
                        .filter(|position| position.is_finite());

                    match kind {
                        MotionKind::Orbit => controller.start_orbit(anchor),
                        MotionKind::Zoom => controller.start_zoom(anchor),
                    }
                    camera_map.insert(*pointer, event.camera());
                }
                Self::End { .. } => {
                    controller.end_move();
                    if let Some(pointer) = camera_map
                        .iter()
                        .find(|(_, camera)| **camera == event.camera())
                        .map(|(pointer, _)| *pointer)
                    {
                        camera_map.remove(&pointer);
                    }
                }
            }
        }
    }

    fn send_pointer_inputs(
        camera_map: Res<CameraPointerMap>,
        mut camera_controllers: Query<&mut PanOrbitCamera>,
        mut mouse_wheel: MessageReader<MouseWheel>,
        mut moves: MessageReader<PointerInput>,
    ) {
        let moves_list: Vec<_> = moves.read().collect();
        for (pointer, camera) in camera_map.iter() {
            let Ok(mut camera_controller) = camera_controllers.get_mut(*camera) else {
                continue;
            };

            let screenspace_input = moves_list
                .iter()
                .filter(|movement| movement.pointer_id.eq(pointer))
                .filter_map(|movement| match movement.action {
                    PointerAction::Move { delta } => Some(delta),
                    _ => None,
                })
                .sum();

            let zoom_amount = match pointer {
                PointerId::Mouse => mouse_wheel
                    .read()
                    .map(|wheel| {
                        let scroll_multiplier = match wheel.unit {
                            MouseScrollUnit::Line => 150.0,
                            MouseScrollUnit::Pixel => 1.0,
                        };
                        wheel.y * scroll_multiplier
                    })
                    .sum::<f32>(),
                _ => 0.0,
            };

            camera_controller.send_screenspace_input(screenspace_input);
            camera_controller.send_zoom_input(zoom_amount);
        }
        mouse_wheel.clear();
    }
}

fn screen_to_view_space(
    camera: &Camera,
    projection: &Projection,
    controller: &PanOrbitCamera,
    target_position: Vec2,
) -> Option<DVec3> {
    let mut viewport_position = if let Some(rect) = camera.logical_viewport_rect() {
        target_position.as_dvec2() - rect.min.as_dvec2()
    } else {
        target_position.as_dvec2()
    };
    let target_size = camera.logical_viewport_size()?.as_dvec2();
    viewport_position.y = target_size.y - viewport_position.y;
    let ndc = viewport_position * 2. / target_size - DVec2::ONE;
    let ndc_to_view = projection.get_clip_from_view().as_dmat4().inverse();
    let view_near_plane = ndc_to_view.project_point3(ndc.extend(1.));

    match projection {
        Projection::Perspective(_) | Projection::Custom(_) => {
            let view_far_plane = ndc_to_view.project_point3(ndc.extend(f64::EPSILON));
            let direction = (view_far_plane - view_near_plane).normalize();
            Some((direction / direction.z) * controller.last_anchor_depth())
        }
        Projection::Orthographic(_) => Some(DVec3::new(
            view_near_plane.x,
            view_near_plane.y,
            controller.last_anchor_depth(),
        )),
    }
}
