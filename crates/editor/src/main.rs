[Reading 1000 lines from start (total: 1473 lines, 473 remaining)]

mod camera_controls;
mod part_editing;
mod runtime_geometry;

use avian3d::prelude::*;
use bevy::camera_controller::pan_orbit_camera::prelude::*;
use bevy::picking::hover::Hovered;
use bevy::prelude::*;
use bevy::text::EditableText;
use bevy::ui_widgets::{Activate, Button, TextInput, observe};
use bevy::window::PrimaryWindow;
use camera_controls::CameraControlsPlugin;
use organism_core::{
    Actuator, ActuatorKind, Behavior, BodyPart, BodyPhysics, ControllerKind, Geometry, Joint,
    JointKind, MotionDefinition, MotionGoal, Organism, RigidBodyProperties, Transform3,
    Vec3 as DataVec3,
};
use organism_serialization::save_project;
use part_editing::{PartEdit, apply_part_edit};
use runtime_geometry::build_runtime_geometry;
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
        .init_resource::<PartInspectorState>()
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

#[derive(Component)]
struct InspectorStatusLabel;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InspectorField {
    Name,
    DimensionA,
    DimensionB,
    DimensionC,
    Density,
    Friction,
    Restitution,
    LinearDamping,
    AngularDamping,
}

#[derive(Component, Clone, Copy)]
struct InspectorFieldInput(InspectorField);

#[derive(Component, Clone, Copy)]
struct InspectorFieldLabel(InspectorField);

#[derive(Resource, Default)]
struct PartInspectorState {
    selected_part_id: Option<String>,
    selected_part: Option<BodyPart>,
}

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
    ApplyPartProperties,
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

macro_rules! inspector_field {
    ($panel:expr, $label:expr, $field:ident) => {
        $panel.spawn((
            Text::new($label),
            TextFont {
                font_size: FontSize::Px(10.0),
                ..default()
            },
            TextColor(Color::srgb(0.72, 0.76, 0.82)),
            InspectorFieldLabel(InspectorField::$field),
        ));
        $panel.spawn((
            TextInput,
            EditableText::new(""),
            Node {
                width: Val::Px(236.0),
                height: Val::Px(32.0),
                padding: UiRect::axes(Val::Px(9.0), Val::Px(5.0)),
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(Color::srgb(0.1, 0.12, 0.16)),
            TextFont {
                font_size: FontSize::Px(13.0),
                ..default()
            },
            TextColor(Color::WHITE),
            InspectorFieldInput(InspectorField::$field),
        ));
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
            inspector_field!(panel, "Part name", Name);
            inspector_field!(panel, "Dimension A", DimensionA);
            inspector_field!(panel, "Dimension B", DimensionB);
            inspector_field!(panel, "Dimension C", DimensionC);
            inspector_field!(panel, "Density ┬╖ kg/m┬│", Density);
            inspector_field!(panel, "Friction coefficient", Friction);
            inspector_field!(panel, "Restitution ┬╖ 0ΓÇô1", Restitution);
            inspector_field!(panel, "Linear damping", LinearDamping);
            inspector_field!(panel, "Angular damping", AngularDamping);
            action_button!(
                panel,
                "Apply part properties",
                EditorButtonAction::ApplyPartProperties,
                Color::srgb(0.23, 0.3, 0.42)
            );
            panel.spawn((
                Text::new("Select a sphere to edit its radius."),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.82, 0.78, 0.57)),
                InspectorStatusLabel,
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
                "Light material ┬╖ 300 kg/m┬│",
                EditorButtonAction::SetDensityLight,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Medium material ┬╖ 1,000 kg/m┬│",
                EditorButtonAction::SetDensityMedium,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Dense material ┬╖ 1,800 kg/m┬│",
                EditorButtonAction::SetDensityDense,
                Color::srgb(0.18, 0.2, 0.27)
            );
            panel.spawn((
                Text::new("Size 1.00├ù ┬╖ Density 1,000 kg/m┬│"),
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
            "Size {:.2}├ù ┬╖ Density {:.0} kg/m┬│",
            placement.size_scale, placement.density_kg_m3
        );
    }
}

fn handle_editor_button_activated(
    activate: On<Activate>,
    actions: Query<&EditorButtonAction>,
    mut tool: ResMut<EditorTool>,
    mut placement: ResMut<PlacementSettings>,
    mut project: ResMut<DraftProject>,
    selection: Res<PartSelection>,
    inspector_inputs: Query<(&EditableText, &InspectorFieldInput)>,
    part_entities: Query<(Entity, &PartTag)>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut inspector_status: Query<&mut Text, With<InspectorStatusLabel>>,
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
        EditorButtonAction::ApplyPartProperties => {
            apply_selected_part_edit(
                selection.part_id.as_deref(),
                &mut project,
                &inspector_inputs,
                &part_entities,
                &mut commands,
                &mut meshes,
                &mut inspector_status,
            );
        }
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

    let Some(runtime_geometry) = build_runtime_geometry(&geometry) else {
        warn!("Cannot construct a runtime shape from invalid geometry");
        return;
    };

    let position = ground_point + Vec3::Y * runtime_geometry.ground_offset;
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
    let mass_properties =
        MassPropertiesBundle::from_shape(&runtime_geometry.collider, properties.density_kg_m3);
    commands.spawn((
        Mesh3d(meshes.add(runtime_geometry.mesh)),
        MeshMaterial3d(materials.add(runtime_geometry.color)),
        Transform::from_translation(position),
        RigidBody::Dynamic,
        runtime_geometry.collider,
        mass_properties,
        Friction::new(properties.friction),
        Restitution::new(properties.restitution),
        LinearDamping(properties.linear_damping),
        AngularDamping(properties.angular_damping),
        PartTag {
            id: part_id.clone(),
            pick_radius: runtime_geometry.pick_radius,
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

--- C:\Users\sfesg\AppData\Local\Temp\tmp69.tmp contents: ---
[Reading 289 lines from start (total: 289 lines, 0 remaining)]

//! Validated, engine-independent body-part editing operations.

use organism_core::{BodyPart, BodyPhysics, Geometry};
use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub struct PartEdit {
    pub name: String,
    pub geometry: Geometry,
    pub density_kg_m3: f32,
    pub friction: f32,
    pub restitution: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartEditError {
    EmptyName,
    GeometryTypeMismatch,
    InvalidGeometry,
    InvalidDensity,
    InvalidFriction,
    InvalidRestitution,
    InvalidDamping,
    UnsupportedPhysics,
}

impl fmt::Display for PartEditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyName => "Part name cannot be empty.",
            Self::GeometryTypeMismatch => {
                "A part's primitive type cannot be changed through dimension fields."
            }
            Self::InvalidGeometry => "Geometry dimensions must be finite and physically valid.",
            Self::InvalidDensity => "Density must be finite and greater than zero kg/m┬│.",
            Self::InvalidFriction => "Friction must be finite and non-negative.",
            Self::InvalidRestitution => "Restitution must be between zero and one.",
            Self::InvalidDamping => "Linear and angular damping must be finite and non-negative.",
            Self::UnsupportedPhysics => "Only rigid body parts can be edited with this inspector.",
        };
        f.write_str(message)
    }
}

impl std::error::Error for PartEditError {}

/// Validates and atomically applies a complete rigid-part edit. Mass is derived
/// from primitive volume and density; failed edits leave the part unchanged.
pub fn apply_part_edit(part: &mut BodyPart, edit: PartEdit) -> Result<(), PartEditError> {
    if edit.name.trim().is_empty() {
        return Err(PartEditError::EmptyName);
    }
    if !same_geometry_kind(&part.geometry, &edit.geometry) {
        return Err(PartEditError::GeometryTypeMismatch);
    }
    if !edit.density_kg_m3.is_finite() || edit.density_kg_m3 <= 0.0 {
        return Err(PartEditError::InvalidDensity);
    }
    if !edit.friction.is_finite() || edit.friction < 0.0 {
        return Err(PartEditError::InvalidFriction);
    }
    if !edit.restitution.is_finite() || !(0.0..=1.0).contains(&edit.restitution) {
        return Err(PartEditError::InvalidRestitution);
    }
    if !edit.linear_damping.is_finite()
        || edit.linear_damping < 0.0
        || !edit.angular_damping.is_finite()
        || edit.angular_damping < 0.0
    {
        return Err(PartEditError::InvalidDamping);
    }
    if !matches!(&part.physics, BodyPhysics::Rigid(_)) {
        return Err(PartEditError::UnsupportedPhysics);
    }
    let mass_kg = edit
        .geometry
        .mass_kg_at_density(edit.density_kg_m3)
        .ok_or(PartEditError::InvalidGeometry)?;

    let BodyPhysics::Rigid(properties) = &mut part.physics else {
        unreachable!("rigid-body type was validated above");
    };
    part.name = edit.name.trim().to_owned();
    part.geometry = edit.geometry;
    properties.mass_kg = mass_kg;
    properties.density_kg_m3 = edit.density_kg_m3;
    properties.friction = edit.friction;
    properties.restitution = edit.restitution;
    properties.linear_damping = edit.linear_damping;
    properties.angular_damping = edit.angular_damping;
    Ok(())
}

fn same_geometry_kind(current: &Geometry, proposed: &Geometry) -> bool {
    matches!(
        (current, proposed),
        (Geometry::Sphere { .. }, Geometry::Sphere { .. })
            | (Geometry::Cuboid { .. }, Geometry::Cuboid { .. })
            | (Geometry::Capsule { .. }, Geometry::Capsule { .. })
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use organism_core::{Organism, RigidBodyProperties, SoftBodyProperties, Transform3, Vec3};

    fn rigid_part(geometry: Geometry) -> BodyPart {
        BodyPart {
            id: "part-0001".into(),
            name: "Body".into(),
            geometry,
            transform: Transform3::default(),
            physics: BodyPhysics::Rigid(RigidBodyProperties {
                density_kg_m3: 950.0,
                friction: 0.65,
                restitution: 0.1,
                linear_damping: 0.03,
                angular_damping: 0.04,
                ..Default::default()
            }),
        }
    }

    fn test_edit(geometry: Geometry) -> PartEdit {
        PartEdit {
            name: "  Edited body  ".into(),
            geometry,
            density_kg_m3: 1200.0,
            friction: 0.8,
            restitution: 0.25,
            linear_damping: 0.1,
            angular_damping: 0.2,
        }
    }

    #[test]
    fn sphere_edit_updates_name_geometry_mass_and_material_properties() {
        let mut part = rigid_part(Geometry::Sphere { radius_m: 0.2 });
        apply_part_edit(&mut part, test_edit(Geometry::Sphere { radius_m: 0.4 }))
            .expect("valid sphere edit");
        assert_eq!(part.name, "Edited body");
        assert_eq!(part.geometry, Geometry::Sphere { radius_m: 0.4 });
        let BodyPhysics::Rigid(p) = part.physics else {
            panic!("expected rigid body");
        };
        let mass = (4.0 / 3.0) * std::f32::consts::PI * 0.4_f32.powi(3) * 1200.0;
        assert!((p.mass_kg - mass).abs() < 1.0e-5);
        assert_eq!(p.density_kg_m3, 1200.0);
        assert_eq!(p.friction, 0.8);
        assert_eq!(p.restitution, 0.25);
        assert_eq!(p.linear_damping, 0.1);
        assert_eq!(p.angular_damping, 0.2);
    }

    #[test]
    fn cuboid_half_extents_edit_recalculates_mass() {
        let mut part = rigid_part(Geometry::Cuboid {
            half_extents_m: Vec3::new(0.2, 0.2, 0.2),
        });
        let geometry = Geometry::Cuboid {
            half_extents_m: Vec3::new(0.3, 0.4, 0.5),
        };
        apply_part_edit(&mut part, test_edit(geometry.clone())).expect("valid cuboid edit");
        assert_eq!(part.geometry, geometry);
        let BodyPhysics::Rigid(p) = part.physics else {
            panic!("expected rigid body");
        };
        assert!((p.mass_kg - 8.0 * 0.3 * 0.4 * 0.5 * 1200.0).abs() < 1.0e-3);
    }

    #[test]
    fn capsule_radius_and_half_height_edit_recalculates_mass() {
        let mut part = rigid_part(Geometry::Capsule {
            radius_m: 0.2,
            half_height_m: 0.3,
        });
        let geometry = Geometry::Capsule {
            radius_m: 0.15,
            half_height_m: 0.45,
        };
        apply_part_edit(&mut part, test_edit(geometry.clone())).expect("valid capsule edit");
        assert_eq!(part.geometry, geometry);
        let BodyPhysics::Rigid(p) = part.physics else {
            panic!("expected rigid body");
        };
        let volume = std::f32::consts::PI * 0.15_f32.powi(2) * 0.9
            + (4.0 / 3.0) * std::f32::consts::PI * 0.15_f32.powi(3);
        assert!((p.mass_kg - volume * 1200.0).abs() < 1.0e-4);
    }

    #[test]
    fn capsule_allows_zero_cylindrical_half_height() {
        let mut part = rigid_part(Geometry::Capsule {
            radius_m: 0.2,
            half_height_m: 0.3,
        });
        apply_part_edit(
            &mut part,
            test_edit(Geometry::Capsule {
                radius_m: 0.25,
                half_height_m: 0.0,
            }),
        )
        .expect("hemisphere-only capsule is valid");
    }

    #[test]
    fn invalid_edits_do_not_partially_mutate_part() {
        let cases = [
            (PartEditError::EmptyName, 0),
            (PartEditError::InvalidDensity, 1),
            (PartEditError::InvalidFriction, 2),
            (PartEditError::InvalidRestitution, 3),
            (PartEditError::InvalidDamping, 4),
            (PartEditError::InvalidGeometry, 5),
        ];
        for (expected, case) in cases {
            let mut part = rigid_part(Geometry::Sphere { radius_m: 0.2 });
            let original = part.clone();
            let mut edit = test_edit(Geometry::Sphere { radius_m: 0.4 });
            match case {
                0 => edit.name = " ".into(),
                1 => edit.density_kg_m3 = f32::NAN,
                2 => edit.friction = -0.1,
                3 => edit.restitution = 1.1,
                4 => edit.angular_damping = f32::INFINITY,
                _ => {
                    edit.geometry = Geometry::Sphere {
                        radius_m: f32::INFINITY,
                    }
                }
            }
            assert_eq!(apply_part_edit(&mut part, edit), Err(expected));
            assert_eq!(part, original);
        }
    }

    #[test]
    fn rejects_geometry_type_changes_and_soft_parts() {
        let mut part = rigid_part(Geometry::Sphere { radius_m: 0.2 });
        let original = part.clone();
        let mismatch = test_edit(Geometry::Cuboid {
            half_extents_m: Vec3::new(0.2, 0.2, 0.2),
        });
        assert_eq!(
            apply_part_edit(&mut part, mismatch),
            Err(PartEditError::GeometryTypeMismatch)
        );
        assert_eq!(part, original);
        part.physics = BodyPhysics::Soft(SoftBodyProperties {
            density_kg_m3: 900.0,
            elastic_modulus_pa: 100_000.0,
            damping_ratio: 0.2,
            resolution: 4,
        });
        let original = part.clone();
        let soft = PartEdit {
            name: part.name.clone(),
            geometry: part.geometry.clone(),
            density_kg_m3: 900.0,
            friction: 0.5,
            restitution: 0.0,
            linear_damping: 0.05,
            angular_damping: 0.05,
        };
        assert_eq!(
            apply_part_edit(&mut part, soft),
            Err(PartEditError::UnsupportedPhysics)
        );
        assert_eq!(part, original);
    }

    #[test]
    fn edited_part_is_encoded_in_the_organism_document() {
        let mut organism = Organism::new("Editable organism");
        let mut part = rigid_part(Geometry::Sphere { radius_m: 0.2 });
        apply_part_edit(&mut part, test_edit(Geometry::Sphere { radius_m: 0.35 }))
            .expect("valid edit");
        organism.parts.push(part);
        let json = organism_serialization::encode_organism(&organism)
            .expect("edited organism should pass validation and encode");
        assert!(json.contains("\"radius_m\": 0.35"));
        assert!(json.contains("\"name\": \"Edited body\""));
        assert!(json.contains("\"density_kg_m3\": 1200.0"));
    }
}