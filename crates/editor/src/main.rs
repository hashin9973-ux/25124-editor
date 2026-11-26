mod camera_controls;

use avian3d::prelude::*;
use bevy::camera_controller::pan_orbit_camera::prelude::*;
use bevy::picking::hover::Hovered;
use bevy::prelude::*;
use bevy::ui_widgets::{Activate, Button, observe};
use bevy::window::PrimaryWindow;
use camera_controls::CameraControlsPlugin;
use organism_core::{
    Actuator, ActuatorKind, Behavior, BodyPart, BodyPhysics, ControllerKind, Geometry, Joint,
    JointKind, MotionDefinition, MotionGoal, Organism, RigidBodyProperties, Transform3,
    Vec3 as DataVec3,
};
use organism_serialization::save_project;
use std::path::Path;

const EXPORT_DIRECTORY: &str = "exports";
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
            CameraControlsPlugin,
        ))
        .insert_resource(ClearColor(Color::srgb(0.025, 0.03, 0.045)))
        .init_resource::<DraftProject>()
        .init_resource::<EditorTool>()
        .init_resource::<PlacementSettings>()
        .init_resource::<PartSelection>()
        .add_systems(Startup, (setup_scene, setup_editor_panel))
        .add_systems(
            Update,
            (
                edit_from_viewport,
                draw_joint_links,
                drive_actuator_behaviors,
                update_placement_settings_label,
                update_part_inspector,
                draw_selected_part_outline,
            ),
        )
        .run();
}

#[derive(Resource)]
struct PlacementSettings {
    size_scale: f32,
    density_kg_m3: f32,
}

impl Default for PlacementSettings {
    fn default() -> Self {
        Self {
            size_scale: 1.0,
            density_kg_m3: 1000.0,
        }
    }
}

#[derive(Component)]
struct PlacementSettingsLabel;

#[derive(Component)]
struct PartInspectorLabel;

/// Selection is stored by stable model ID rather than by a transient ECS entity.
#[derive(Resource, Default)]
struct PartSelection {
    part_id: Option<String>,
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
    Select,
    PlaceCuboid,
    PlaceCapsule,
    ConnectFixed,
    ConnectBall,
    ConnectHinge,
    ConnectPoweredHinge,
    ConnectSlider,
    ConnectPoweredSlider,
    ConnectDistance,
}

impl EditorTool {
    fn joint_kind(self) -> Option<JointKind> {
        match self {
            Self::ConnectFixed => Some(JointKind::Fixed),
            Self::ConnectBall => Some(JointKind::Ball),
            Self::ConnectHinge | Self::ConnectPoweredHinge => Some(JointKind::Hinge),
            Self::ConnectSlider | Self::ConnectPoweredSlider => Some(JointKind::Slider),
            Self::ConnectDistance => Some(JointKind::Distance),
            Self::Select | Self::PlaceSphere | Self::PlaceCuboid | Self::PlaceCapsule => None,
        }
    }

    fn is_powered_hinge(self) -> bool {
        self == Self::ConnectPoweredHinge
    }

    fn is_powered_slider(self) -> bool {
        self == Self::ConnectPoweredSlider
    }

    fn geometry(self, size_scale: f32) -> Option<Geometry> {
        match self {
            Self::PlaceSphere => Some(Geometry::Sphere {
                radius_m: 0.3 * size_scale,
            }),
            Self::PlaceCuboid => Some(Geometry::Cuboid {
                half_extents_m: DataVec3::new(
                    0.25 * size_scale,
                    0.25 * size_scale,
                    0.25 * size_scale,
                ),
            }),
            Self::PlaceCapsule => Some(Geometry::Capsule {
                radius_m: 0.2 * size_scale,
                half_height_m: 0.45 * size_scale,
            }),
            Self::ConnectFixed
            | Self::ConnectBall
            | Self::ConnectHinge
            | Self::ConnectPoweredHinge
            | Self::ConnectSlider
            | Self::ConnectPoweredSlider
            | Self::ConnectDistance
            | Self::Select => None,
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
    SelectPart,
    PlaceCuboid,
    PlaceCapsule,
    ConnectFixed,
    ConnectBall,
    ConnectHinge,
    ConnectPoweredHinge,
    ConnectSlider,
    ConnectPoweredSlider,
    ConnectDistance,
    SetSizeSmall,
    SetSizeStandard,
    SetSizeLarge,
    SetDensityLight,
    SetDensityMedium,
    SetDensityDense,
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
                observe(handle_editor_button_activated),
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
            Hovered::default(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(16.0),
                left: Val::Px(16.0),
                width: Val::Px(270.0),
                height: Val::Vh(90.0),
                overflow: Overflow {
                    x: OverflowAxis::Hidden,
                    y: OverflowAxis::Scroll,
                },
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
                Text::new("PART INSPECTOR"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));

            action_button!(
                panel,
                "Select / inspect part",
                EditorButtonAction::SelectPart,
                Color::srgb(0.27, 0.22, 0.12)
            );
            panel.spawn((
                Text::new("No part selected. Choose Select and click a body part."),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(Color::srgb(0.88, 0.86, 0.72)),
                PartInspectorLabel,
            ));

            panel.spawn((
                Text::new("BODY CONSTRUCTION"),
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
                Text::new("ASSISTED DEFAULTS"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));
            action_button!(
                panel,
                "Small parts",
                EditorButtonAction::SetSizeSmall,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Standard parts",
                EditorButtonAction::SetSizeStandard,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Large parts",
                EditorButtonAction::SetSizeLarge,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Light material · 300 kg/m³",
                EditorButtonAction::SetDensityLight,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Medium material · 1,000 kg/m³",
                EditorButtonAction::SetDensityMedium,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Dense material · 1,800 kg/m³",
                EditorButtonAction::SetDensityDense,
                Color::srgb(0.18, 0.2, 0.27)
            );
            panel.spawn((
                Text::new("Size 1.00× · Density 1,000 kg/m³"),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(Color::srgb(0.85, 0.83, 0.64)),
                PlacementSettingsLabel,
            ));

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
                "Connect powered slider",
                EditorButtonAction::ConnectPoweredSlider,
                Color::srgb(0.36, 0.25, 0.12)
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

fn update_placement_settings_label(
    placement: Res<PlacementSettings>,
    mut labels: Query<&mut Text, With<PlacementSettingsLabel>>,
) {
    if !placement.is_changed() {
        return;
    }

    for mut label in &mut labels {
        label.0 = format!(
            "Size {:.2}× · Density {:.0} kg/m³",
            placement.size_scale, placement.density_kg_m3
        );
    }
}

fn handle_editor_button_activated(
    activate: On<Activate>,
    actions: Query<&EditorButtonAction>,
    mut tool: ResMut<EditorTool>,
    mut placement: ResMut<PlacementSettings>,
    project: Res<DraftProject>,
) {
    let Ok(action) = actions.get(activate.entity) else {
        return;
    };

    match *action {
        EditorButtonAction::PlaceSphere => *tool = EditorTool::PlaceSphere,
        EditorButtonAction::SelectPart => *tool = EditorTool::Select,
        EditorButtonAction::PlaceCuboid => *tool = EditorTool::PlaceCuboid,
        EditorButtonAction::PlaceCapsule => *tool = EditorTool::PlaceCapsule,
        EditorButtonAction::ConnectFixed => *tool = EditorTool::ConnectFixed,
        EditorButtonAction::ConnectBall => *tool = EditorTool::ConnectBall,
        EditorButtonAction::ConnectHinge => *tool = EditorTool::ConnectHinge,
        EditorButtonAction::ConnectPoweredHinge => *tool = EditorTool::ConnectPoweredHinge,
        EditorButtonAction::ConnectSlider => *tool = EditorTool::ConnectSlider,
        EditorButtonAction::ConnectPoweredSlider => *tool = EditorTool::ConnectPoweredSlider,
        EditorButtonAction::ConnectDistance => *tool = EditorTool::ConnectDistance,
        EditorButtonAction::SetSizeSmall => placement.size_scale = 0.5,
        EditorButtonAction::SetSizeStandard => placement.size_scale = 1.0,
        EditorButtonAction::SetSizeLarge => placement.size_scale = 2.0,
        EditorButtonAction::SetDensityLight => placement.density_kg_m3 = 300.0,
        EditorButtonAction::SetDensityMedium => placement.density_kg_m3 = 1000.0,
        EditorButtonAction::SetDensityDense => placement.density_kg_m3 = 1800.0,
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
fn edit_from_viewport(
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    hovered_panels: Query<&Hovered>,
    part_query: Query<(Entity, &GlobalTransform, &PartTag)>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut project: ResMut<DraftProject>,
    tool: Res<EditorTool>,
    placement: Res<PlacementSettings>,
    mut selection: ResMut<PartSelection>,
) {
    if !buttons.just_pressed(MouseButton::Left) {
        return;
    }
    if hovered_panels.iter().any(|hovered| hovered.get()) {
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

    if *tool == EditorTool::Select {
        let picked = pick_body_part(ray.origin, ray_direction, &part_query)
            .map(|(_, _, part_id, _)| part_id);

        if selection.part_id != picked {
            match &picked {
                Some(part_id) => info!("Selected {part_id}"),
                None => info!("Selection cleared"),
            }
            selection.part_id = picked;
        }
        return;
    }

    if let Some(geometry) = tool.geometry(placement.size_scale) {
        place_body_part(
            geometry,
            ray.origin,
            ray_direction,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut project,
            placement.density_kg_m3,
        );
        return;
    }

    let Some(joint_kind) = tool.joint_kind() else {
        return;
    };
    connect_body_parts(
        joint_kind,
        tool.is_powered_hinge(),
        tool.is_powered_slider(),
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
    density_kg_m3: f32,
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
            let half_extents = Vec3::new(half_extents_m.x, half_extents_m.y, half_extents_m.z);
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
    let Some(estimated_mass_kg) = geometry.mass_kg_at_density(density_kg_m3) else {
        warn!("Cannot place a part with invalid geometry or estimated mass");
        return;
    };
    let properties = RigidBodyProperties {
        mass_kg: estimated_mass_kg,
        density_kg_m3,
        friction: PART_FRICTION,
        restitution: PART_RESTITUTION,
        linear_damping: PART_LINEAR_DAMPING,
        angular_damping: PART_ANGULAR_DAMPING,
    };

    // Avian expects density here and derives mass/inertia from the collider volume.
    let mass_properties = MassPropertiesBundle::from_shape(&collider, properties.density_kg_m3);
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

fn pick_body_part(
    ray_origin: Vec3,
    ray_direction: Vec3,
    part_query: &Query<(Entity, &GlobalTransform, &PartTag)>,
) -> Option<(f32, Entity, String, GlobalTransform)> {
    part_query
        .iter()
        .filter_map(|(entity, transform, tag)| {
            ray_sphere_distance(
                ray_origin,
                ray_direction,
                transform.translation(),
                tag.pick_radius * 1.08,
            )
            .map(|distance| (distance, entity, tag.id.clone(), *transform))
        })
        .min_by(|left, right| left.0.total_cmp(&right.0))
}

fn connect_body_parts(
    kind: JointKind,
    powered_hinge: bool,
    powered_slider: bool,
    ray_origin: Vec3,
    ray_direction: Vec3,
    part_query: &Query<(Entity, &GlobalTransform, &PartTag)>,
    commands: &mut Commands,
    project: &mut DraftProject,
) {
    let picked = pick_body_part(ray_origin, ray_direction, part_query);

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
        .map(|(entity, transform, _)| (entity, *transform))
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
            commands
                .spawn(SphericalJoint::new(parent_entity, child_entity).with_anchor(world_anchor));
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
        JointKind::Slider if powered_slider => {
            let joint_id = format!("joint-{:04}", project.next_joint_id);
            let motor = LinearMotor::new(MotorModel::SpringDamper {
                frequency: 8.0,
                damping_ratio: 1.0,
            })
            .with_target_position(0.0)
            .with_max_force(20.0);
            commands.spawn((
                PrismaticJoint::new(parent_entity, child_entity)
                    .with_anchor(world_anchor)
                    .with_slider_axis(Vec3::X)
                    .with_motor(motor),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
            commands.entity(parent_entity).insert(SleepingDisabled);
            commands.entity(child_entity).insert(SleepingDisabled);
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

    if powered_hinge || powered_slider {
        let actuator_sequence = project.next_actuator_id;
        project.next_actuator_id += 1;
        let actuator_id = format!("actuator-{actuator_sequence:04}");
        let (actuator_kind, max_force_n, max_torque_nm, default_amplitude, name) = if powered_hinge
        {
            (
                ActuatorKind::AngularMotor,
                None,
                Some(10.0),
                0.55,
                format!("Powered hinge {actuator_sequence}"),
            )
        } else {
            (
                ActuatorKind::LinearMotor,
                Some(20.0),
                None,
                0.2,
                format!("Powered slider {actuator_sequence}"),
            )
        };
        project.organism.actuators.push(Actuator {
            id: actuator_id.clone(),
            name,
            joint_id: joint_id.clone(),
            kind: actuator_kind,
            max_force_n,
            max_torque_nm,
        });
        project.motion.behaviors.push(Behavior {
            id: format!("behavior-{actuator_sequence:04}"),
            name: format!("Oscillate {joint_id}"),
            controller: ControllerKind::Oscillator,
            goals: vec![MotionGoal::OscillateActuator {
                actuator_id,
                frequency_hz: 0.5,
                amplitude: default_amplitude,
            }],
        });
    }
    info!("Created {kind:?} connection");
}

fn drive_actuator_behaviors(
    time: Res<Time>,
    project: Res<DraftProject>,
    mut angular_joints: Query<(&mut RevoluteJoint, &RuntimeJointTag)>,
    mut linear_joints: Query<(&mut PrismaticJoint, &RuntimeJointTag)>,
) {
    let elapsed = time.elapsed_secs();
    for behavior in &project.motion.behaviors {
        for goal in &behavior.goals {
            let MotionGoal::OscillateActuator {
                actuator_id,
                frequency_hz,
                amplitude,
            } = goal
            else {
                continue;
            };
            let Some(actuator) = project
                .organism
                .actuators
                .iter()
                .find(|actuator| actuator.id == *actuator_id)
            else {
                continue;
            };
            let target = amplitude * (std::f32::consts::TAU * frequency_hz * elapsed).sin();

            match actuator.kind {
                ActuatorKind::AngularMotor => {
                    for (mut joint, tag) in &mut angular_joints {
                        if tag.id == actuator.joint_id {
                            joint.motor.target_position = target;
                            break;
                        }
                    }
                }
                ActuatorKind::LinearMotor => {
                    for (mut joint, tag) in &mut linear_joints {
                        if tag.id == actuator.joint_id {
                            joint.motor.target_position = target;
                            break;
                        }
                    }
                }
                ActuatorKind::MuscleTendon => {}
            }
        }
    }
}

fn update_part_inspector(
    selection: Res<PartSelection>,
    project: Res<DraftProject>,
    mut labels: Query<&mut Text, With<PartInspectorLabel>>,
) {
    if !selection.is_changed() && !project.is_changed() {
        return;
    }

    let details = selection
        .part_id
        .as_ref()
        .and_then(|part_id| {
            project
                .organism
                .parts
                .iter()
                .find(|part| part.id == *part_id)
        })
        .map(describe_body_part)
        .unwrap_or_else(|| {
            "No part selected. Choose Select / inspect part, then click a body in the viewport."
                .to_owned()
        });

    for mut label in &mut labels {
        label.0.clone_from(&details);
    }
}

fn describe_body_part(part: &BodyPart) -> String {
    let geometry = match &part.geometry {
        Geometry::Sphere { radius_m } => format!("Sphere · radius {radius_m:.3} m"),
        Geometry::Cuboid { half_extents_m } => format!(
            "Cuboid · half-extents ({:.3}, {:.3}, {:.3}) m",
            half_extents_m.x, half_extents_m.y, half_extents_m.z
        ),
        Geometry::Capsule {
            radius_m,
            half_height_m,
        } => format!("Capsule · radius {radius_m:.3} m · half-height {half_height_m:.3} m"),
    };

    let physics = match &part.physics {
        BodyPhysics::Rigid(properties) => format!(
            "Rigid body\nMass: {:.3} kg\nDensity: {:.1} kg/m³\nFriction: {:.2}\nRestitution: {:.2}\nLinear damping: {:.3}\nAngular damping: {:.3}",
            properties.mass_kg,
            properties.density_kg_m3,
            properties.friction,
            properties.restitution,
            properties.linear_damping,
            properties.angular_damping
        ),
        BodyPhysics::Soft(properties) => format!(
            "Soft body\nDensity: {:.1} kg/m³\nElastic modulus: {:.1} Pa\nDamping ratio: {:.3}\nResolution: {}",
            properties.density_kg_m3,
            properties.elastic_modulus_pa,
            properties.damping_ratio,
            properties.resolution
        ),
    };

    format!(
        "{} ({})\nGeometry: {}\nRest position: ({:.3}, {:.3}, {:.3}) m\n{}",
        part.name,
        part.id,
        geometry,
        part.transform.translation_m.x,
        part.transform.translation_m.y,
        part.transform.translation_m.z,
        physics
    )
}

fn draw_selected_part_outline(
    mut gizmos: Gizmos,
    selection: Res<PartSelection>,
    parts: Query<(&GlobalTransform, &PartTag)>,
) {
    let Some(selected_id) = selection.part_id.as_deref() else {
        return;
    };

    if let Some((transform, tag)) = parts.iter().find(|(_, tag)| tag.id == selected_id) {
        gizmos.sphere(
            transform.translation(),
            tag.pick_radius * 1.16,
            Color::srgb(1.0, 0.82, 0.28),
        );
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
