use avian3d::prelude::*;
use bevy::camera_controller::pan_orbit_camera::prelude::*;
use bevy::prelude::*;
use organism_core::{
    Actuator, ActuatorKind, Behavior, BodyPart, BodyPhysics, ControllerKind, Geometry, Joint,
    JointKind, MotionDefinition, MotionGoal, Organism, RigidBodyProperties, Transform3,
    Vec3 as DataVec3,
};
use organism_serialization::save_project;
use std::path::Path;

const EXPORT_DIRECTORY: &str = "exports";
const PART_MASS_KG: f32 = 1.0;
const PART_FRICTION: f32 = 0.75;
const PART_RESTITUTION: f32 = 0.0;
const PART_LINEAR_DAMPING: f32 = 0.03;
const PART_ANGULAR_DAMPING: f32 = 0.05;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            MeshPickingPlugin,
            DefaultPanOrbitCameraPlugins,
        ))
        .insert_resource(ClearColor(Color::srgb(0.025, 0.03, 0.045)))
        .init_resource::<DraftProject>()
        .init_resource::<EditorTool>()
        .add_systems(Startup, (setup_scene, setup_editor_panel))
        .add_systems(
            Update,
            (
                handle_editor_buttons,
                edit_from_viewport,
                draw_joint_links,
                drive_actuator_behaviors,
            ),
        )
        .run();
}

#[derive(Resource)]
struct DraftProject {
    organism: Organism,
    motion: MotionDefinition,
    next_part_id: u64,
    next_joint_id: u64,
    next_actuator_id: u64,
    pending_joint_part: Option<String>,
}

impl Default for DraftProject {
    fn default() -> Self {
        let organism = Organism::new("Untitled organism");
        let motion = MotionDefinition::new(organism.id.clone());
        Self {
            organism,
            motion,
            next_part_id: 1,
            next_joint_id: 1,
            next_actuator_id: 1,
            pending_joint_part: None,
        }
    }
}

#[derive(Resource, Default, Clone, Copy, PartialEq, Eq)]
enum EditorTool {
    #[default]
    PlaceSphere,
    PlaceCuboid,
    PlaceCapsule,
    ConnectFixed,
    ConnectBall,
    ConnectHinge,
    ConnectPoweredHinge,
    ConnectSlider,
    ConnectDistance,
}

impl EditorTool {
    fn joint_kind(self) -> Option<JointKind> {
        match self {
            Self::ConnectFixed => Some(JointKind::Fixed),
            Self::ConnectBall => Some(JointKind::Ball),
            Self::ConnectHinge | Self::ConnectPoweredHinge => Some(JointKind::Hinge),
            Self::ConnectSlider => Some(JointKind::Slider),
            Self::ConnectDistance => Some(JointKind::Distance),
            Self::PlaceSphere | Self::PlaceCuboid | Self::PlaceCapsule => None,
        }
    }

    fn is_powered_hinge(self) -> bool {
        self == Self::ConnectPoweredHinge
    }

    fn geometry(self) -> Option<Geometry> {
        match self {
            Self::PlaceSphere => Some(Geometry::Sphere { radius_m: 0.3 }),
            Self::PlaceCuboid => Some(Geometry::Cuboid {
                half_extents_m: DataVec3::new(0.25, 0.25, 0.25),
            }),
            Self::PlaceCapsule => Some(Geometry::Capsule {
                radius_m: 0.2,
                half_height_m: 0.45,
            }),
            Self::ConnectFixed
            | Self::ConnectBall
            | Self::ConnectHinge
            | Self::ConnectPoweredHinge
            | Self::ConnectSlider
            | Self::ConnectDistance => None,
        }
    }
}

#[derive(Component)]
struct RuntimeJointTag {
    id: String,
}

#[derive(Component)]
struct PartTag {
    id: String,
    pick_radius: f32,
}

#[derive(Component, Clone, Copy)]
enum EditorButtonAction {
    PlaceSphere,
    PlaceCuboid,
    PlaceCapsule,
    ConnectFixed,
    ConnectBall,
    ConnectHinge,
    ConnectPoweredHinge,
    ConnectSlider,
    ConnectDistance,
    Save,
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let floor_size = 24.0;
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(floor_size, 0.2, floor_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.11, 0.13, 0.17))),
        Transform::from_xyz(0.0, -0.1, 0.0),
        RigidBody::Static,
        Collider::cuboid(floor_size * 0.5, 0.1, floor_size * 0.5),
        Friction::new(0.85),
        Restitution::new(0.0),
    ));

    commands.spawn((
        PointLight {
            intensity: 2_000_000.0,
            range: 80.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 10.0, 5.0),
    ));

    commands.spawn((
        Camera3d::default(),
        PanOrbitCamera {
            enabled_motion: EnabledMotion {
                pan: false,
                orbit: true,
                zoom: true,
            },
            ..default()
        },
        Transform::from_xyz(9.0, 8.0, 12.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
}

macro_rules! action_button {
    ($panel:expr, $label:expr, $action:expr, $color:expr) => {
        $panel
            .spawn((
                Button,
                Node {
                    width: Val::Px(236.0),
                    height: Val::Px(34.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor($color),
                $action,
            ))
            .with_children(|button| {
                button.spawn((
                    Text::new($label),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
    };
}

fn setup_editor_panel(mut commands: Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(16.0),
                left: Val::Px(16.0),
                width: Val::Px(260.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(7.0),
                padding: UiRect::all(Val::Px(12.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.035, 0.045, 0.065, 0.96)),
        ))
        .with_children(|panel| {
            panel.spawn((
                Text::new("ORGANISM STUDIO"),
                TextFont {
                    font_size: FontSize::Px(18.0),
                    ..default()
                },
                TextColor(Color::WHITE),
            ));
            panel.spawn((
                Text::new("BODY CONSTRUCTION"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));

            action_button!(
                panel,
                "Add sphere",
                EditorButtonAction::PlaceSphere,
                Color::srgb(0.12, 0.2, 0.3)
            );
            action_button!(
                panel,
                "Add cuboid",
                EditorButtonAction::PlaceCuboid,
                Color::srgb(0.12, 0.2, 0.3)
            );
            action_button!(
                panel,
                "Add capsule",
                EditorButtonAction::PlaceCapsule,
                Color::srgb(0.12, 0.2, 0.3)
            );

            panel.spawn((
                Text::new("CONNECTIONS"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));

            action_button!(
                panel,
                "Connect fixed",
                EditorButtonAction::ConnectFixed,
                Color::srgb(0.16, 0.19, 0.26)
            );
            action_button!(
                panel,
                "Connect ball-and-socket",
                EditorButtonAction::ConnectBall,
                Color::srgb(0.16, 0.19, 0.26)
            );
            action_button!(
                panel,
                "Connect hinge",
                EditorButtonAction::ConnectHinge,
                Color::srgb(0.16, 0.19, 0.26)
            );
            action_button!(
                panel,
                "Connect powered hinge (oscillating)",
                EditorButtonAction::ConnectPoweredHinge,
                Color::srgb(0.36, 0.25, 0.12)
            );
            action_button!(
                panel,
                "Connect slider",
                EditorButtonAction::ConnectSlider,
                Color::srgb(0.16, 0.19, 0.26)
            );
            action_button!(
                panel,
                "Connect distance",
                EditorButtonAction::ConnectDistance,
                Color::srgb(0.16, 0.19, 0.26)
            );

            panel.spawn((
                Text::new("EXPORT"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));
            action_button!(
                panel,
                "Save organism + motion JSON",
                EditorButtonAction::Save,
                Color::srgb(0.16, 0.34, 0.23)
            );

            panel.spawn((
                Text::new(
                    "Place: click the ground\nConnect: select two parts\nJoints connect at their midpoint\nUnits: meters / kilograms / seconds",
                ),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(Color::srgb(0.72, 0.76, 0.82)),
            ));
        });
}

fn handle_editor_buttons(
    interactions: Query<(&Interaction, &EditorButtonAction), (Changed<Interaction>, With<Button>)>,
    mut tool: ResMut<EditorTool>,
    project: Res<DraftProject>,
) {
    for (interaction, action) in &interactions {
        if *interaction != Interaction::Pressed {
            continue;
        }

        match action {
            EditorButtonAction::PlaceSphere => *tool = EditorTool::PlaceSphere,
            EditorButtonAction::PlaceCuboid => *tool = EditorTool::PlaceCuboid,
            EditorButtonAction::PlaceCapsule => *tool = EditorTool::PlaceCapsule,
            EditorButtonAction::ConnectFixed => *tool = EditorTool::ConnectFixed,
            EditorButtonAction::ConnectBall => *tool = EditorTool::ConnectBall,
            EditorButtonAction::ConnectHinge => *tool = EditorTool::ConnectHinge,
            EditorButtonAction::ConnectPoweredHinge => {
                *tool = EditorTool::ConnectPoweredHinge
            }
            EditorButtonAction::ConnectSlider => *tool = EditorTool::ConnectSlider,
            EditorButtonAction::ConnectDistance => *tool = EditorTool::ConnectDistance,
            EditorButtonAction::Save => {
                match save_project(
                    Path::new(EXPORT_DIRECTORY),
                    &project.organism,
                    &project.motion,
                ) {
                    Ok(paths) => info!(
                        "Saved organism to {} and motion to {}",
                        paths.organism.display(),
                        paths.motion.display()
                    ),
                    Err(error) => error!("Could not save project: {error}"),
                }
            }
        }
    }
}

fn edit_from_viewport(
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    ui_interactions: Query<&Interaction, With<Button>>,
    part_query: Query<(Entity, &GlobalTransform, &PartTag)>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut project: ResMut<DraftProject>,
    tool: Res<EditorTool>,
) {
    if !buttons.just_pressed(MouseButton::Left) {
        return;
    }
    if ui_interactions
        .iter()
        .any(|interaction| *interaction != Interaction::None)
    {
        return;
    }

    let Ok(window) = windows.single() else {
        return;
    };
    let Some(cursor_position) = window.cursor_position() else {
        return;
    };
    let Ok((camera, camera_transform)) = cameras.single() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
        return;
    };
    let ray_direction = ray.direction.as_vec3();

    if let Some(geometry) = tool.geometry() {
        place_body_part(
            geometry,
            ray.origin,
            ray_direction,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut project,
        );
        return;
    }

    let Some(joint_kind) = tool.joint_kind() else {
        return;
    };
    connect_body_parts(
        joint_kind,
        tool.is_powered_hinge(),
        ray.origin,
        ray_direction,
        &part_query,
        &mut commands,
        &mut project,
    );
}

fn place_body_part(
    geometry: Geometry,
    ray_origin: Vec3,
    ray_direction: Vec3,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    project: &mut DraftProject,
) {
    if ray_direction.y.abs() < 1.0e-5 {
        return;
    }
    let ground_distance = -ray_origin.y / ray_direction.y;
    if ground_distance <= 0.0 {
        return;
    }
    let ground_point = ray_origin + ray_direction * ground_distance;

    let (mesh, collider, pick_radius, ground_offset, color) = match &geometry {
        Geometry::Sphere { radius_m } => (
            meshes.add(Sphere::new(*radius_m)),
            Collider::sphere(*radius_m),
            *radius_m,
            *radius_m,
            Color::srgb(0.24, 0.68, 0.92),
        ),
        Geometry::Cuboid { half_extents_m } => {
            let half_extents =
                Vec3::new(half_extents_m.x, half_extents_m.y, half_extents_m.z);
            (
                meshes.add(Cuboid::from_size(half_extents * 2.0)),
                Collider::cuboid(half_extents.x, half_extents.y, half_extents.z),
                half_extents.length(),
                half_extents.y,
                Color::srgb(0.32, 0.78, 0.58),
            )
        }
        Geometry::Capsule {
            radius_m,
            half_height_m,
        } => {
            let full_segment_length = half_height_m * 2.0;
            let outer_radius = radius_m + half_height_m;
            (
                meshes.add(Capsule3d::new(*radius_m, full_segment_length)),
                Collider::capsule(*radius_m, full_segment_length),
                outer_radius,
                outer_radius,
                Color::srgb(0.92, 0.63, 0.28),
            )
        }
    };

    let position = ground_point + Vec3::Y * ground_offset;
    let sequence = project.next_part_id;
    project.next_part_id += 1;
    let part_id = format!("part-{sequence:04}");
    let properties = RigidBodyProperties {
        mass_kg: PART_MASS_KG,
        friction: PART_FRICTION,
        restitution: PART_RESTITUTION,
        linear_damping: PART_LINEAR_DAMPING,
        angular_damping: PART_ANGULAR_DAMPING,
    };

    let mass_properties = MassPropertiesBundle::from_shape(&collider, properties.mass_kg);
    commands.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(materials.add(color)),
        Transform::from_translation(position),
        RigidBody::Dynamic,
        collider,
        mass_properties,
        Friction::new(properties.friction),
        Restitution::new(properties.restitution),
        LinearDamping(properties.linear_damping),
        AngularDamping(properties.angular_damping),
        PartTag {
            id: part_id.clone(),
            pick_radius,
        },
    ));

    project.organism.parts.push(BodyPart {
        id: part_id.clone(),
        name: format!("Part {sequence}"),
        geometry,
        transform: Transform3::at(DataVec3::new(position.x, position.y, position.z)),
        physics: BodyPhysics::Rigid(properties),
    });
    info!("Added {part_id}");
}

fn connect_body_parts(
    kind: JointKind,
    powered_hinge: bool,
    ray_origin: Vec3,
    ray_direction: Vec3,
    part_query: &Query<(Entity, &GlobalTransform, &PartTag)>,
    commands: &mut Commands,
    project: &mut DraftProject,
) {
    let picked = part_query
        .iter()
        .filter_map(|(entity, transform, tag)| {
            let position = transform.translation();
            ray_sphere_distance(ray_origin, ray_direction, position, tag.pick_radius * 1.08)
                .map(|distance| (distance, entity, tag.id.clone(), transform))
        })
        .min_by(|left, right| left.0.total_cmp(&right.0));

    let Some((_, child_entity, child_id, child_transform)) = picked else {
        project.pending_joint_part = None;
        info!("No body part under cursor; select a body part");
        return;
    };

    let Some(parent_id) = project.pending_joint_part.take() else {
        project.pending_joint_part = Some(child_id.clone());
        info!("Selected {child_id}; now select the second body part");
        return;
    };

    if parent_id == child_id {
        project.pending_joint_part = Some(child_id);
        info!("Choose a different body part for the second endpoint");
        return;
    }

    let Some((parent_entity, parent_transform)) = part_query
        .iter()
        .find(|(_, _, tag)| tag.id == parent_id)
        .map(|(entity, transform, _)| (entity, transform))
    else {
        warn!("Selected parent part {parent_id} no longer exists");
        return;
    };

    let parent_position = parent_transform.translation();
    let child_position = child_transform.translation();
    let center_distance = parent_position.distance(child_position);
    let world_anchor = (parent_position + child_position) * 0.5;

    match kind {
        JointKind::Fixed => {
            commands.spawn(FixedJoint::new(parent_entity, child_entity).with_anchor(world_anchor));
        }
        JointKind::Ball => {
            commands.spawn(
                SphericalJoint::new(parent_entity, child_entity).with_anchor(world_anchor),
            );
        }
        JointKind::Hinge if powered_hinge => {
            let joint_id = format!("joint-{:04}", project.next_joint_id);
            let motor = AngularMotor::new(MotorModel::SpringDamper {
                frequency: 8.0,
                damping_ratio: 1.0,
            })
            .with_target_position(0.0)
            .with_max_torque(10.0);
            commands.spawn((
                RevoluteJoint::new(parent_entity, child_entity)
                    .with_anchor(world_anchor)
                    .with_hinge_axis(Vec3::Z)
                    .with_motor(motor),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
            commands.entity(parent_entity).insert(SleepingDisabled);
            commands.entity(child_entity).insert(SleepingDisabled);
        }
        JointKind::Hinge => {
            commands.spawn(
                RevoluteJoint::new(parent_entity, child_entity)
                    .with_anchor(world_anchor)
                    .with_hinge_axis(Vec3::Z),
            );
        }
        JointKind::Slider => {
            commands.spawn(
                PrismaticJoint::new(parent_entity, child_entity)
                    .with_anchor(world_anchor)
                    .with_slider_axis(Vec3::X),
            );
        }
        JointKind::Distance => {
            commands.spawn(
                DistanceJoint::new(parent_entity, child_entity)
                    .with_limits(center_distance.max(0.001), center_distance.max(0.001)),
            );
        }
    }

    let joint_sequence = project.next_joint_id;
    project.next_joint_id += 1;

    let (anchor_parent, anchor_child, rest_length_m) = if kind == JointKind::Distance {
        (Vec3::ZERO, Vec3::ZERO, Some(center_distance.max(0.001)))
    } else {
        (
            parent_transform
                .affine()
                .inverse()
                .transform_point3(world_anchor),
            child_transform
                .affine()
                .inverse()
                .transform_point3(world_anchor),
            None,
        )
    };

    let joint_id = format!("joint-{joint_sequence:04}");
    project.organism.joints.push(Joint {
        id: joint_id.clone(),
        name: format!("{parent_id} to {child_id} {kind:?}"),
        kind,
        parent_part_id: parent_id,
        child_part_id: child_id,
        anchor_parent_m: DataVec3::new(anchor_parent.x, anchor_parent.y, anchor_parent.z),
        anchor_child_m: DataVec3::new(anchor_child.x, anchor_child.y, anchor_child.z),
        limits: None,
        rest_length_m,
    });

    if powered_hinge {
        let actuator_sequence = project.next_actuator_id;
        project.next_actuator_id += 1;
        let actuator_id = format!("actuator-{actuator_sequence:04}");
        project.organism.actuators.push(Actuator {
            id: actuator_id.clone(),
            name: format!("Powered hinge {actuator_sequence}"),
            joint_id: joint_id.clone(),
            kind: ActuatorKind::AngularMotor,
            max_force_n: None,
            max_torque_nm: Some(10.0),
        });
        project.motion.behaviors.push(Behavior {
            id: format!("behavior-{actuator_sequence:04}"),
            name: format!("Oscillate {joint_id}"),
            controller: ControllerKind::Oscillator,
            goals: vec![MotionGoal::OscillateActuator {
                actuator_id,
                frequency_hz: 0.5,
                amplitude: 0.55,
            }],
        });
    }
    info!("Created {kind:?} connection");
}

fn drive_actuator_behaviors(
    time: Res<Time>,
    project: Res<DraftProject>,
    mut joints: Query<(&mut RevoluteJoint, &RuntimeJointTag)>,
) {
    let elapsed = time.elapsed_secs();
    for (mut joint, runtime_tag) in &mut joints {
        let Some(actuator) = project
            .organism
            .actuators
            .iter()
            .find(|actuator| actuator.joint_id == runtime_tag.id)
        else {
            continue;
        };

        let goal = project
            .motion
            .behaviors
            .iter()
            .flat_map(|behavior| behavior.goals.iter())
            .find_map(|goal| match goal {
                MotionGoal::OscillateActuator {
                    actuator_id,
                    frequency_hz,
                    amplitude,
                } if actuator_id == &actuator.id => Some((*frequency_hz, *amplitude)),
                _ => None,
            });

        if let Some((frequency_hz, amplitude)) = goal {
            joint.motor.target_position =
                amplitude * (std::f32::consts::TAU * frequency_hz * elapsed).sin();
        }
    }
}

fn draw_joint_links(
    mut gizmos: Gizmos,
    part_query: Query<(&GlobalTransform, &PartTag)>,
    project: Res<DraftProject>,
) {
    for joint in &project.organism.joints {
        let parent = part_query
            .iter()
            .find(|(_, tag)| tag.id == joint.parent_part_id)
            .map(|(transform, _)| transform.translation());
        let child = part_query
            .iter()
            .find(|(_, tag)| tag.id == joint.child_part_id)
            .map(|(transform, _)| transform.translation());

        if let (Some(parent), Some(child)) = (parent, child) {
            gizmos.line(parent, child, Color::srgb(0.95, 0.78, 0.32));
        }
    }
}

fn ray_sphere_distance(
    ray_origin: Vec3,
    ray_direction: Vec3,
    sphere_center: Vec3,
    sphere_radius: f32,
) -> Option<f32> {
    let offset = ray_origin - sphere_center;
    let b = offset.dot(ray_direction);
    let c = offset.length_squared() - sphere_radius * sphere_radius;
    let discriminant = b * b - c;
    if discriminant < 0.0 {
        return None;
    }

    let near = -b - discriminant.sqrt();
    let far = -b + discriminant.sqrt();
    if near >= 0.0 {
        Some(near)
    } else if far >= 0.0 {
        Some(far)
    } else {
        None
    }
}
